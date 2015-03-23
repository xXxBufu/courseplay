--[[
@title:     Course Generation for Courseplay
@authors:   Jakob Tischler, Fck54 (Franck Champlon)
@version:   2.00
@date:      10 Jan 2015

@copyright: No reproduction, usage or copying without the explicit permission by the authors allowed.
]]--

local geometry = courseplay.geometry;
local abs, sqrt = math.abs, math.sqrt;

function courseplay:generateCourse(vehicle)
	local self = courseplay.generation;
	-----------------------------------
	local workWidth = vehicle.cp.workWidth;
	local epsilon = vehicle.cp.fieldEdge.douglasPeuckerEpsilon or 2;
	local startPoint = vehicle.cp.startingPointCoordsManual;
	-----------------------------------
	if not vehicle.cp.overlap then vehicle.cp.overlap = 1/4 end; --TODO add this in the menu
	if not vehicle.cp.headland.minPointDistance then vehicle.cp.headland.minPointDistance = .5 end;
	if not vehicle.cp.headland.maxPointDistance then vehicle.cp.headland.maxPointDistance = 5 end;

	local fieldCourseName = tostring(vehicle.cp.currentCourseName);
	if vehicle.cp.fieldEdge.selectedField.fieldNum > 0 then
		fieldCourseName = courseplay.fields.fieldData[vehicle.cp.fieldEdge.selectedField.fieldNum].name;
	end;
	courseplay:debug(string.format("generateCourse() called for %q", fieldCourseName), 7);

	-- Make sure everything's set and in order
	courseplay:validateCourseGenerationData(vehicle);
	if not vehicle.cp.hasValidCourseGenerationData then
		return;
	end;

	local field = {};
	if vehicle.cp.fieldEdge.selectedField.fieldNum > 0 then
		field = courseplay.utils.table.copy(courseplay.fields.fieldData[vehicle.cp.fieldEdge.selectedField.fieldNum].points, true);
	else
		field = courseplay.utils.table.copy(vehicle.Waypoints, true);
	end;
	field = geometry:setPolyCounterClockwise(field);

	local pointsInField = #(field);
	local polyPoints = field;

	courseplay:debug(string.format('before headland: field = %s,  pointsInField = %d', tostring(field), pointsInField), 7);

	courseplay:clearCurrentLoadedCourse(vehicle);

	---#################################################################
	-- (1) SET UP CORNERS AND DIRECTIONS --
	--------------------------------------------------------------------
	courseplay:debug('(1) SET UP CORNERS AND DIRECTIONS', 7);
	local ridgeMarker = {
		none = 0,
		left = 1,
		right = 2
	};
	local orderCW = vehicle.cp.headland.userDirClockwise;
	local numLanes = vehicle.cp.headland.numLanes;
	local turnAround = vehicle.cp.headland.turnAround or numLanes > 49;
	local numHeadlandLanesCreated ;

	---#################################################################
	-- (2) HEADLAND
	--------------------------------------------------------------------
	courseplay:debug('(2) HEADLAND', 7);

	if numLanes and numLanes > 0 then --we have headland, baby!

		courseplay:debug(string.format("generateCourse(%i): headland.numLanes=%s, headland.orderBefore=%s, turnAround = %s", debug.getinfo(1).currentline, tostring(vehicle.cp.headland.numLanes), tostring(vehicle.cp.headland.orderBefore), tostring(turnAround)), 7);

		vehicle.cp.headland.lanes = {};
		local offsetWidth = 0;
		local curLane = 1;

		while curLane <= numLanes or turnAround do

			-- set ridge marker side --
			local laneRidgeMarker = ridgeMarker.none;
			if numLanes > 1 then
				if vehicle.cp.headland.orderBefore and curLane < numLanes then
					laneRidgeMarker = orderCW and ridgeMarker.right or ridgeMarker.left;
				elseif not vehicle.cp.headland.orderBefore and curLane > 1 then
					laneRidgeMarker = orderCW and ridgeMarker.left or ridgeMarker.right;
				end;
			end;

			local offsetWidth = self:getOffsetWidth(vehicle, curLane);

			courseplay:debug(string.format('headland lane %d: laneRidgeMarker=%d, offset offsetWidth=%.1f', curLane, laneRidgeMarker, offsetWidth), 7);

			-- --------------------------------------------------
			-- (2.1) CREATE INITIAL OFFSET POINTS
			courseplay:debug('(2.1) CREATE INITIAL OFFSET POINTS', 7);
			local lane = geometry:offsetPoly(polyPoints, -offsetWidth, vehicle);
			if not lane then 
				turnAround = false;
				break;
			end;
			polyPoints = courseplay.utils.table.copy(lane,true); -- save result for next lane and field lanes calculation.
			if not(orderCW) then 
				lane = table.reverse(lane);
			end;

			-- --------------------------------------------------
			-- (2.2) FINALIZE (ADD POINT DATA)
			local numOffsetPoints, data = #lane, {};
			if numOffsetPoints > 2 then
				for i, point in ipairs(lane) do
					if i < numOffsetPoints then
						point.angle = geometry:signAngleDeg(point,lane[i+1]);
					else
						point.angle = geometry:signAngleDeg(point,lane[1]);
					end;
					if i > 1 then
						--courseplay:debug(string.format('headland - lane %d (data.angle = %s, point.angle = %s)', curLane, tostring(data.angle), tostring(point.angle)),7);
					end;
					data = {
						cx = point.cx,
						cy = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, point.cx, 1, point.cz), --TODO: actually only needed for debugPoint/debugLine
						cz = point.cz,
						angle = point.angle,
						wait = false,
						rev = false,
						crossing = false,
						generated = true,
						lane = curLane * -1, --negative lane = headland
						-- cp.firstInLane = false;
						turn = nil,
						turnStart = false,
						turnEnd = false,
						ridgeMarker = laneRidgeMarker
						};
					lane[i] = data;
				end;

				courseplay:debug(string.format("generateCourse(%i): #lane %s = %s", debug.getinfo(1).currentline, tostring(curLane), tostring(numOffsetPoints)), 7);
				--courseplay:debug(tableShow(lane, string.format('[line %d] lane %d', debug.getinfo(1).currentline, curLane), 7), 7); -- WORKS

				-- --------------------------------------------------
				-- (2.3) FINALIZE (ADD LANE TO HEADLAND)
				table.insert(vehicle.cp.headland.lanes, lane);
				courseplay:debug(string.format("generateCourse(%i): inserting lane #%d (%d points) into headland.lanes", debug.getinfo(1).currentline, curLane, numOffsetPoints), 7);
				-- courseplay:debug(tableShow(lane, string.format('[line %d] lane %d', debug.getinfo(1).currentline, curLane), 7), 7); --WORKS
				curLane = curLane + 1;
			else
				courseplay:debug(string.format('headland lane #%d has fewer than 2 points (invalid) -> stop headland calculation', curLane), 7);
				turnAround = true;
				break;
			end;
		end; --END while curLane in numLanes


		numHeadlandLanesCreated = #(vehicle.cp.headland.lanes);
		courseplay:debug(string.format('generateCourse(%i):  #vehicle.cp.headland.lanes=%s', debug.getinfo(1).currentline, tostring(numHeadlandLanesCreated)), 7);
		-- courseplay:debug(tableShow(vehicle.cp.headland.lanes, 'vehicle.cp.headland.lanes', 7), 7); --WORKS

		-- -- --------------------------------------------------
		-- -- (2.4) SETTING FIELD WORK COURSE BASE
		-- if numHeadlandLanesCreated > 0 and not(turnAround) then
			-- if vehicle.cp.overlap ~= 0 then
				-- offsetWidth = self:getOffsetWidth(vehicle, numHeadlandLanesCreated + 1);
				-- offsetWidth = (offsetWidth / 2) * (1 - vehicle.cp.overlap);
				-- polyPoints = geometry:offsetPoly(polyPoints, -offsetWidth);
			-- end;
			-- numPoints = #(polyPoints);
			-- courseplay:debug(string.format('headland: numHeadlandLanesCreated=%d, workArea has %s points', numHeadlandLanesCreated, tostring(numPoints)), 7);
		-- end;

	end; --END if vehicle.cp.headland.numLanes ~= 0


	---#################################################################
	-- (3) DIMENSIONS, ALL PATH POINTS
	--------------------------------------------------------------------


	------------------------------------------------------------------
	-- (3.1) DEF POINT IN FIELD CORNER FOR TURN AROUND
	-------------------------------------------------------------------
	-- (3.2) SETTING FIELD WORK COURSE BASE
	courseplay:debug('(3) DIMENSIONS, ALL PATH POINTS', 7);

	local fieldWorkCourse, fieldPoints = {}, {};
	local _, _, dimensions, _ = geometry:getPolygonData(field, nil, nil, true, true, false);
	courseplay:debug(string.format('minX=%s, maxX=%s', tostring(dimensions.minX), tostring(dimensions.maxX)), 7); --WORKS
	courseplay:debug(string.format('minZ=%s, maxZ=%s', tostring(dimensions.minZ), tostring(dimensions.maxZ)), 7); --WORKS
	courseplay:debug(string.format('generateCourse(%i): width=%s, height=%s', debug.getinfo(1).currentline, tostring(dimensions.width), tostring(dimensions.height)), 7); --WORKS
	
	if not turnAround then
		fieldWorkCourse = geometry:genWorkCourse(polyPoints, vehicle, startPoint);
	end;

	---############################################################################
	-- (5) ROTATE HEADLAND COURSES
	-------------------------------------------------------------------------------
	courseplay:debug('(5) ROTATE HEADLAND COURSES', 7);
	if vehicle.cp.headland.numLanes > 0 then
		courseplay:debug(string.format('[headlands] rotating headland, number of lanes = %d', numHeadlandLanesCreated),7);
		if numHeadlandLanesCreated > 0 then
			-- -------------------------------------------------------------------------
			-- (5.1) ROTATE AND LINK HEADLAND LANES
			if vehicle.cp.headland.orderBefore or turnAround then --each headland lane 's first point is closest to corresponding field begin point or previous lane end point
				local lanes = {};
				local prevLane = {};
				for i,lane in ipairs(vehicle.cp.headland.lanes) do
					local numPoints = #lane;
					courseplay:debug(string.format('current lane has %d points',numPoints),7);
					local closest = geometry:getClosestPolyPoint(lane, startPoint.cx, startPoint.cz);
					courseplay:debug(string.format('closest point on lane is : %s (%.2f, %.2f)', tostring(closest), lane[closest].cx, lane[closest].cz), 7);
					courseplay:debug(string.format('[before] rotating headland lane=%d, closest=%d -> rotate: numPoints-(closest)=%d-(%d-1)=%d', i, closest, numPoints, closest, numPoints - (closest-1)), 7);
			        lane = table.rotate(lane, numPoints - (closest-1));
					if i > 1 then -- link lanes
						local p3 = lane[1];
						local p4 = lane[2];
						local idx = #prevLane;
						local p2 = prevLane[idx];
						local side = orderCW and 'left' or 'right';
						if geometry:getAreAnglesClose(p3.angle,p2.angle,20) then
							table.remove(prevLane);
							idx = idx - 1;
							p2 = prevLane[idx];
						else
							while geometry:getPointSide(p2, p3, p4) == side do
								table.remove(prevLane);
								idx = idx - 1;
								p2 = prevLane[idx];
							end;
						end;
						if geometry:getAreAnglesOpposite(p3.angle,p2.angle,10) then
						    lane[1].turnEnd = true;
						    prevLane[idx].turnStart = true;
						    prevLane[idx].turn = orderCW and 'right' or 'left';
						else
							local p1 = prevLane[idx-1];
							local crossing = geometry:lineIntersection(p1,p2,p3,p4);
							if crossing.ip1 == 'PFIP' and crossing.ip2 == 'NFIP' and not(geometry:getAreAnglesClose(p2.angle,p3.angle,30)) then
							crossing.angle = geometry:signAngleDeg(p2,crossing);
								local data = {
									cx = crossing.cx,
									cy = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, crossing.cx, 1, crossing.cz), --TODO: actually only needed for debugPoint/debugLine
									cz = crossing.cz,
									angle = crossing.angle,
									wait = false,
									rev = false,
									crossing = false,
									generated = true,
									lane = p2.lane, --negative lane = headland
									-- cp.firstInLane = false;
									turn = nil,
									turnStart = false,
									turnEnd = false,
									ridgeMarker = p2.ridgeMarker
								};
								table.insert(prevLane,data);
							end;
						end;
						--prevLane[idx].turnStart = true; --debug
						--lane[1].turnEnd = true; --debug
						table.remove(lanes);
						table.insert(lanes,prevLane);
						table.remove(lane);
						numPoints = #lane;
						startPoint = lane[1];
					end;
					if i == numHeadlandLanesCreated and not(turnAround) then -- last headlane link to first fieldWorkCourse point
					end;
					if i == 1 then
						-- set start point on the field edge
						startPoint.cx = lane[1].cx;
						startPoint.cz = lane[1].cz;
						local newStart = geometry:findCrossing(lane[3],lane[2], field, 'PFIP');
						local dx,dz,_ = geometry:getPointDirection(lane[3],lane[2],true);
						lane[1].cx = newStart.cx + (1*dx);
						lane[1].cz = newStart.cz + (1*dz);
						lane[1].cy = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, lane[1].cx, 1, lane[1].cz)
						table.remove(lane);
					end;
					prevLane = lane;
					table.insert(lanes,lane);
				end;
				vehicle.cp.headland.lanes = nil;
				vehicle.cp.headland.lanes = {};
				vehicle.cp.headland.lanes = lanes;

			elseif #fieldWorkCourse and not vehicle.cp.headland.orderBefore then --each headland lanes' first point is closest to last fieldwork course point
				local lastFieldworkPoint = fieldWorkCourse[#fieldWorkCourse];
				--courseplay:debug(tableShow(lastFieldworkPoint, 'lastFieldworkPoint'), 7); --TODO: is nil - whyyyyy?
				local lanes = {} ;
				local prevLane = {};
				for i = numHeadlandLanesCreated, 1, -1 do
					local lane = vehicle.cp.headland.lanes[i];
					local numPoints = #lane;
					local closest = geometry:getClosestPolyPoint(lane, lastFieldworkPoint.cx, lastFieldworkPoint.cz); --TODO: works, but how if lastFieldWorkPoint is nil???
					courseplay:debug(string.format('[after] rotating headland lane=%d, closest=%d -> rotate: numPoints-(closest-1)=%d-(%d-1)=%d', i, closest, numPoints, closest, numPoints - (closest-1)), 7);
					lane = table.rotate(lane, numPoints - (closest - 1));
					if i < numHeadlandLanesCreated then -- link lanes
						local p3 = lane[1];
						local p4 = lane[2];
						local idx = #prevLane;
						local p2 = prevLane[idx];
						local side = orderCW and 'left' or 'right';
						if geometry:getAreAnglesClose(p3.angle,p2.angle,20) then
							table.remove(prevLane);
							idx = idx - 1;
							p2 = prevLane[idx];
						else
							while geometry:getPointSide(p2, p3, p4) == side do
								table.remove(prevLane);
								idx = idx - 1;
								p2 = prevLane[idx];
							end;
						end;
						if geometry:getAreAnglesOpposite(p3.angle,p2.angle,10) then
						    lane[1].turnEnd = true;
						    prevLane[idx].turnStart = true;
						    prevLane[idx].turn = orderCW and 'right' or 'left';
						else
							local p1 = prevLane[idx-1];
							local crossing = geometry:lineIntersection(p1,p2,p3,p4);
							if crossing.ip1 == 'PFIP' and crossing.ip2 == 'NFIP' and not(geometry:getAreAnglesClose(p2.angle,p3.angle,10)) then
							crossing.angle = geometry:signAngleDeg(p2,crossing);
								local data = {
									cx = crossing.cx,
									cy = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, crossing.cx, 1, crossing.cz), --TODO: actually only needed for debugPoint/debugLine
									cz = crossing.cz,
									angle = crossing.angle,
									wait = false,
									rev = false,
									crossing = false,
									generated = true,
									lane = p2.lane, --negative lane = headland
									-- cp.firstInLane = false;
									turn = nil,
									turnStart = false,
									turnEnd = false,
									ridgeMarker = p2.ridgeMarker
								};
								table.insert(prevLane,data);
							end;
						end;
						--prevLane[idx].turnStart = true; --debug
						--lane[1].turnEnd = true; --debug
						table.remove(lanes);
						table.insert(lanes,prevLane);
						numPoints = #lane;
						startPoint = lane[1];
					end;
					if i == numHeadlandLanesCreated and geometry:getAreAnglesClose(geometry:signAngleDeg(lane[1],lane[2]) + 180,lastFieldworkPoint.angle, 5, 'deg') then
						--we will have to make a turn maneuver before entering headland
						Lane[1].turnEnd = true;
						fieldWorkCourse[#fieldWorkCourse].turnStart = true;
						if orderCW then
							fieldWorkCourse[#fieldWorkCourse].turn = 'right';
						else
							fieldWorkCourse[#fieldWorkCourse].turn = 'left';
						end;
					end;
					lastFieldWorkPoint = lane[numPoints];
					prevLane = lane ;
					table.insert(lanes, lane);
				end;

				vehicle.cp.headland.lanes = nil;
				vehicle.cp.headland.lanes = {};
				vehicle.cp.headland.lanes = lanes;
				--courseplay:debug(tableShow(vehicle.cp.headland.lanes, 'rotated headland lanes'), 7);
			end;
		end;
	end;

	---############################################################################
	-- (7) CONCATENATE HEADLAND COURSE and FIELDWORK COURSE
	-------------------------------------------------------------------------------
	courseplay:debug('(7) CONCATENATE HEADLAND COURSE and FIELDWORK COURSE', 7);

	vehicle.Waypoints = {};

	if vehicle.cp.headland.numLanes > 0 then
		if vehicle.cp.headland.orderBefore then
			for i=1, #(vehicle.cp.headland.lanes) do
				vehicle.Waypoints = tableConcat(vehicle.Waypoints, vehicle.cp.headland.lanes[i]);
			end;
			vehicle.Waypoints = tableConcat(vehicle.Waypoints, fieldWorkCourse);
		else
			vehicle.Waypoints = tableConcat(vehicle.Waypoints, fieldWorkCourse);
			for i=1, #(vehicle.cp.headland.lanes) do
				vehicle.Waypoints = tableConcat(vehicle.Waypoints, vehicle.cp.headland.lanes[i]);
			end;
		end;
	else
		vehicle.Waypoints = fieldWorkCourse;
	end;
	local returnToStart = {};
	if vehicle.cp.returnToFirstPoint then
		vehicle.cp.numWaypoints = #(vehicle.Waypoints);
		vehicle.Waypoints[vehicle.cp.numWaypoints].wait = false;

		local p1 = vehicle.Waypoints[vehicle.cp.numWaypoints-1];
		local p2 = vehicle.Waypoints[vehicle.cp.numWaypoints];
		local p3 = vehicle.Waypoints[2];
		local p4 = vehicle.Waypoints[1];
		local dist = Utils.vector2Length(p2.cx-p3.cx,p2.cz-p3.cz);
		returnToStart = geometry:smoothSpline(p1, p2, p3, p4, dist/vehicle.cp.headland.maxPointDistance, true, true, false);
		table.remove(returnToStart, 1);
		local data = {};
		for i, point in pairs(returnToStart) do
			if  i < #returnToStart then
				point.angle = geometry:signAngleDeg(point,returnToStart[i+1]);
			else
				point.angle = data.angle;
			end;
			data = {
				cx = point.cx,
				cy = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, point.cx, 1, point.cz), --TODO: actually only needed for debugPoint/debugLine
				cz = point.cz,
				angle = point.angle,
				wait = false,
				rev = false,
				crossing = false,
				generated = true,
				lane = p2.lane, --negative lane = headland
				-- cp.firstInLane = false;
				turn = nil,
				turnStart = false,
				turnEnd = false,
				ridgeMarker = 0
				};
			returnToStart[i] = data;
		end;
		p4.angle = geometry:invertAngleDeg(p4.angle);
		table.insert(returnToStart,p4);

		-- local srcCourse = fieldWorkCourse;
		-- if vehicle.cp.headland.numLanes and vehicle.cp.headland.numLanes > 0 and vehicle.cp.headland.orderBefore and #(vehicle.cp.headland.lanes) > 0 then
			-- srcCourse = vehicle.cp.headland.lanes[1];
			-- courseplay:debug(string.format('lastFivePoints: #headland.lanes=%d, headland.orderBefore=%s -> srcCourse = headland.lanes[1]', #(vehicle.cp.headland.lanes), tostring(vehicle.cp.headland.orderBefore)), 7);
		-- end;

		-- for b=5, 1, -1 do
			-- local origPathPoint = srcCourse[b];

			-- local point = {
				-- cx = origPathPoint.cx,
				-- cz = origPathPoint.cz,
				-- angle = geometry:invertAngleDeg(origPathPoint.angle),
				-- wait = false, --b == 1,
				-- rev = false,
				-- crossing = false,
				-- lane = 1,
				-- turnStart = false,
				-- turnEnd = false,
				-- ridgeMarker = 0,
				-- generated = true
			-- };
			-- table.insert(lastFivePoints, point);
		-- end;
	end;

	if #(returnToStart) > 0 then
		vehicle.Waypoints = tableConcat(vehicle.Waypoints, returnToStart);
	end;



	---############################################################################
	-- (7) FINAL COURSE DATA
	-------------------------------------------------------------------------------
	courseplay:debug('(7) FINAL COURSE DATA', 7);
	vehicle.cp.numWaypoints = #(vehicle.Waypoints)
	if vehicle.cp.numWaypoints == 0 then
		courseplay:debug('ERROR: #vehicle.Waypoints == 0 -> cancel and return', 7);
		return;
	end;

	vehicle.cp.waypointIndex = 1;
	vehicle.cp.canDrive = true;
	vehicle.Waypoints[1].wait = true;
	vehicle.Waypoints[1].crossing = true;
	vehicle.Waypoints[vehicle.cp.numWaypoints].wait = true;
	vehicle.Waypoints[vehicle.cp.numWaypoints].crossing = true;
	vehicle.cp.numCourses = 1;
	courseplay.signs:updateWaypointSigns(vehicle);

	-- extra data for turn maneuver
	vehicle.cp.courseWorkWidth = vehicle.cp.workWidth;
	vehicle.cp.courseNumHeadlandLanes = numHeadlandLanesCreated;
	vehicle.cp.courseHeadlandDirectionCW = vehicle.cp.headland.userDirClockwise;

	vehicle.cp.hasGeneratedCourse = true;
	courseplay:setFieldEdgePath(vehicle, nil, 0);
	courseplay:validateCourseGenerationData(vehicle);
	courseplay:validateCanSwitchMode(vehicle);
	courseplay:debug(string.format("generateCourse() finished: %d lanes, %d headland", numLanes, vehicle.cp.headland.lanes and #vehicle.cp.headland.lanes or 0), 7);

	-- SETUP 2D COURSE DRAW DATA
	vehicle.cp.course2dUpdateDrawData = true;

	-- TURN OFF GENERATION PREVIEW
	-- [TODO]
end;



-- ####################################################################################################


function courseplay.generation:getOffsetWidth(vehicle, laneNum)
	local w = vehicle.cp.workWidth;
	if laneNum == 1 then
		w = w * 0.5;
	end;
	return w;
end;



-- ####################################################################################################
