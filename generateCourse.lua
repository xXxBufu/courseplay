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
		local remainingPoints = geometry:simplifyPolygon(field, epsilon);
		local field = geometry:newPolyFromIndices(field, remainingPoints); -- make new polygon
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

	local corners = {
		[1] = 'SW',
		[2] = 'NW',
		[3] = 'NE',
		[4] = 'SE'
	};
	local directions = {
		[1] = 'N',
		[2] = 'E',
		[3] = 'S',
		[4] = 'W'
	};
	local ridgeMarker = {
		none = 0,
		left = 1,
		right = 2
	};
	local crn = corners[vehicle.cp.startingCorner];
	local dir = directions[vehicle.cp.startingDirection];
	local orderCW = vehicle.cp.headland.userDirClockwise;
	local numLanes = vehicle.cp.headland.numLanes;
	local turnAround = vehicle.cp.headland.turnAround or numLanes > 49;
	local numHeadLanes ;

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
						courseplay:debug(string.format('headland - lane %d (data.angle = %s, point.angle = %s)', curLane, tostring(data.angle), tostring(point.angle)),7);
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


		numHeadLanes = #(vehicle.cp.headland.lanes);
		courseplay:debug(string.format('generateCourse(%i):  #vehicle.cp.headland.lanes=%s', debug.getinfo(1).currentline, tostring(numHeadLanes)), 7);
		-- courseplay:debug(tableShow(vehicle.cp.headland.lanes, 'vehicle.cp.headland.lanes', 7), 7); --WORKS

		-- --------------------------------------------------
		-- (2.4) SETTING FIELD WORK COURSE BASE
		if numHeadLanes > 0 and not(turnAround) then
			if vehicle.cp.overlap ~= 0 then
				offsetWidth = self:getOffsetWidth(vehicle, numHeadLanes + 1);
				offsetWidth = (offsetWidth / 2) * (1 - vehicle.cp.overlap);
				polyPoints = geometry:offsetPoly(polyPoints, -offsetWidth);
			end;
			numPoints = #(polyPoints);
			courseplay:debug(string.format('headland: numHeadLanes=%d, workArea has %s points', numHeadLanes, tostring(numPoints)), 7);
		end;

	end; --END if vehicle.cp.headland.numLanes ~= 0


	---#################################################################
	-- (3) DIMENSIONS, ALL PATH POINTS
	--------------------------------------------------------------------
	local StartPoint = {};

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

	local numLanes, pointsPerLane = 0, 0;
	local curLaneDir = '';
	local pointDistance = 5;
	local pipSafety = 0.1;
	local pathPoints = {};

	if dir == "N" or dir == "S" then --North or South
		numLanes = math.ceil(dimensions.width / workWidth);
		pointsPerLane = math.ceil(dimensions.height / pointDistance);
		if (numLanes * workWidth) < dimensions.width then
			numLanes = numLanes + 1;
		end;
		courseplay:debug(string.format('generateCourse(%i): numLanes=%s, pointsPerLane=%s', debug.getinfo(1).currentline, tostring(numLanes), tostring(pointsPerLane)), 7); --WORKS

		local gotALane = false;
		for curLane=1, numLanes do
			--Lane directions
			if gotALane then
				if curLaneDir == "N" then
					curLaneDir = "S";
				else
					curLaneDir = "N";
				end;
			else
				curLaneDir = dir;
			end;
			courseplay:debug(string.format("curLane = %d, curLaneDir = %s", curLane, curLaneDir), 7); --WORKS

			for a=1, pointsPerLane do
				local curPoint = {
					num = a + ((curLane-1) * pointsPerLane),
					lane = curLane,
					laneDir = curLaneDir,
					cx = dimensions.minX + (workWidth * curLane) - (workWidth/2),
					cz = dimensions.minZ
				};

				if crn == "NW" or crn == "SW" then
					if curLaneDir == "S" then
						curPoint.ridgeMarker = ridgeMarker["left"];
					else
						curPoint.ridgeMarker = ridgeMarker["right"];
					end;
				elseif crn == "SE" or crn == "NE" then
					if curLaneDir == "S" then
						curPoint.ridgeMarker = ridgeMarker["right"];
					else
						curPoint.ridgeMarker = ridgeMarker["left"];
					end;
				end;

				if crn == "NE" or crn == "SE" then
					curPoint.cx = dimensions.maxX - (workWidth * curLane) + (workWidth/2);
				end;

				if curLaneDir == "S" then
					curPoint.cz = dimensions.minZ + (pointDistance * (a-1));

					if curPoint.cz >= dimensions.maxZ then
						curPoint.cz = dimensions.maxZ - pipSafety;
					end;
				elseif curLaneDir == "N" then
					curPoint.cz = dimensions.maxZ - (pointDistance * (a-1));

					if curPoint.cz <= dimensions.minZ then
						curPoint.cz = dimensions.minZ + pipSafety;
					end;
				end;

				--last lane
				curPoint.cx = Utils.clamp(curPoint.cx, dimensions.minX + (workWidth/2), dimensions.maxX - (workWidth/2));

				--is point in field?
				local _, pointInPoly = geometry:minDistToPline(curPoint, polyPoints);
				if pointInPoly then
					courseplay:debug(string.format("Point %d (lane %d, point %d) - x=%.1f, z=%.1f - in Poly - adding to pathPoints", curPoint.num, curLane, a, curPoint.cx, curPoint.cz), 7);
					table.insert(pathPoints, curPoint);
					gotALane = true;
				else
					courseplay:debug(string.format("Point %d (lane %d, point %d) - x=%.1f, z=%.1f - not in Poly - not adding to pathPoints", curPoint.num, curLane, a, curPoint.cx, curPoint.cz), 7);
				end;
				_, pointInPoly = geometry:minDistToPline(curPoint, field);
				if pointInPoly then
				    table.insert(fieldPoints, curPoint);
				end;

			end; --END for curPoint in pointsPerLane
		end; --END for curLane in numLanes
	--END North or South

	elseif dir == "E" or dir == "W" then --East or West
		numLanes = math.ceil(dimensions.height / workWidth);
		pointsPerLane = math.ceil(dimensions.width / pointDistance);
		if numLanes * workWidth < dimensions.height then
			numLanes = numLanes + 1;
		end;
		courseplay:debug(string.format("generateCourse(%i): numLanes = %s, pointsPerLane = %s", debug.getinfo(1).currentline, tostring(numLanes), tostring(pointsPerLane)), 7); --WORKS

		local gotALane = false;
		for curLane=1, numLanes do
			--Lane directions
			if gotALane then
				if curLaneDir == "E" then
					curLaneDir = "W";
				else
					curLaneDir = "E";
				end;
			else
				curLaneDir = dir;
			end;

			courseplay:debug(string.format("curLane = %d, curLaneDir = %s", curLane, curLaneDir), 7); --WORKS

			for a=1, pointsPerLane do
				local curPoint = {
					num = a + ((curLane-1) * pointsPerLane),
					lane = curLane,
					laneDir = curLaneDir,
					cx = dimensions.minX,
					cz = dimensions.minZ + (workWidth * curLane) - (workWidth/2)
				};

				if crn == "SW" or crn == "SE" then
					if curLaneDir == "E" then
						curPoint.ridgeMarker = ridgeMarker["left"];
					else
						curPoint.ridgeMarker = ridgeMarker["right"];
					end;
				elseif crn == "NE" or crn == "NW" then
					if curLaneDir == "E" then
						curPoint.ridgeMarker = ridgeMarker["right"];
					else
						curPoint.ridgeMarker = ridgeMarker["left"];
					end;
				end;

				if crn == "SW" or crn == "SE" then
					curPoint.cz = dimensions.maxZ - (workWidth * curLane) + (workWidth/2);
				end;

				if curLaneDir == "E" then
					curPoint.cx = dimensions.minX + (pointDistance * (a-1));

					if curPoint.cx >= dimensions.maxX then
						curPoint.cx = dimensions.maxX - pipSafety;
					end;
				elseif curLaneDir == "W" then
					curPoint.cx = dimensions.maxX - (pointDistance * (a-1));

					if curPoint.cx <= dimensions.minX then
						curPoint.cx = dimensions.minX + pipSafety;
					end;
				end;

				--last lane
				curPoint.z = Utils.clamp(curPoint.cz, dimensions.minZ + (workWidth/2), dimensions.maxZ - (workWidth/2));

				--is point in field?
				local _, pointInPoly = geometry:minDistToPline(curPoint, polyPoints);
				if pointInPoly then
					courseplay:debug(string.format("Point %d (lane %d, point %d) - x=%.1f, z=%.1f - in Poly - adding to pathPoints", curPoint.num, curLane, a, curPoint.cx, curPoint.cz), 7);
					table.insert(pathPoints, curPoint);
					gotALane = true;
				else
					courseplay:debug(string.format("Point %d (lane %d, point %d) - x=%.1f, z=%.1f - not in Poly - not adding to pathPoints", curPoint.num, curLane, a, curPoint.cx, curPoint.cz), 7);
				end;
				_, pointInPoly = geometry:minDistToPline(curPoint, field);
				if pointInPoly then
				    table.insert(fieldPoints, curPoint);
				end;


			end; --END for curPoint in pointsPerLane
		end; --END for curLane in numLanes
	end; --END East or West

	---############################################################################
	-- (4) CHECK PATH LANES FOR VALID START AND END POINTS and FILL fieldWorkCourse
	-------------------------------------------------------------------------------
	courseplay:debug('(4) CHECK PATH LANES FOR VALID START AND END POINTS and FILL fieldWorkCourse', 7);
	local numPoints = #pathPoints;
	for i=1, numPoints do
		local cp = pathPoints[i];   --current
		local np = pathPoints[i+1]; --next
		local pp = pathPoints[i-1]; --previous

		if i == 1 then
			pp = pathPoints[numPoints];
		end;
		if i == numPoints then
			np = pathPoints[1];
		end;

		cp.firstInLane = pp.lane ~= cp.lane; --previous point in different lane -> I'm first in lane
		cp.lastInLane = np.lane ~= cp.lane; --next point in different lane -> I'm last in lane
		local isLastLane = cp.lane == numLanes;


		--REAL ANGLE: right = 0deg, top = 90deg, left = 180deg, bottom = 270deg
		--SIGN ANGLE: N=180, E=90, S=0, W=270
		local angleDeg, signAngleDeg;
		if cp.firstInLane or i == 1 then
			angleDeg     = geometry:angleDeg(cp, np);
			signAngleDeg = geometry:signAngleDeg(cp, np);
		else
			angleDeg     = geometry:angleDeg(pp, cp);
			signAngleDeg = geometry:signAngleDeg(pp, cp);
		end;
		
		if cp.firstInLane or i == 1 or isLastLane then
			cp.ridgeMarker = 0;
		end;

		local point = {
			cx = cp.cx,
			cz = cp.cz,
			angle = angleDeg,
			wait = nil, --will be set to true for first and last after all is set and done
			rev = nil,
			crossing = nil,
			lane = cp.lane,
			laneDir = cp.laneDir,
			turnStart = courseplay:trueOrNil(cp.lastInLane and cp.lane < numLanes),
			turnEnd = courseplay:trueOrNil(cp.firstInLane and i > 1),
			ridgeMarker = cp.ridgeMarker,
			generated = true
		};

		local newFirstInLane, newLastInLane;

		--TURN MANEUVER ... AND STUFF
		if cp.firstInLane then
			newFirstInLane = geometry:findCrossing(np, cp, polyPoints, "PFIP"); -- search the intersection point before cp

			if newFirstInLane ~= nil then
				--courseplay:debug(string.format("lane %d: newFirstInLane: x=%f, z=%f", cp.lane, newFirstInLane.x, newFirstInLane.z), 7);

				newFirstInLane.angle = point.angle;
				newFirstInLane.wait = point.wait;
				newFirstInLane.crossing = point.crossing;
				newFirstInLane.rev = point.rev;
				newFirstInLane.lane = point.lane;
				newFirstInLane.laneDir = point.laneDir;
				newFirstInLane.firstInLane = true;
				newFirstInLane.turn = point.turn;
				newFirstInLane.turnStart = nil;
				newFirstInLane.turnEnd = courseplay:trueOrNil(i > 1);
				newFirstInLane.ridgeMarker = 0;
				newFirstInLane.generated = true;

				--reset some vars in old first point
				point.wait = nil;
				point.firstInLane = false;
				point.turn = nil;
				point.turnStart = nil;
				point.turnEnd = nil;
			end;
		end; --END cp.firstInLane

		if cp.lastInLane then

			--North
			if cp.laneDir == "N" then
				if np.cx < cp.cx then point.turn = "left" end;
				if np.cx > cp.cx then point.turn = "right" end;

			--East
			elseif cp.laneDir == "E" then
				if np.cz < cp.cz then point.turn = "left" end;
				if np.cz > cp.cz then point.turn = "right" end;

			--South
			elseif cp.laneDir == "S" then
				if np.cx < cp.cx then point.turn = "right" end;
				if np.cx > cp.cx then point.turn = "left" end;

			--West
			elseif cp.laneDir == "W" then
				if np.cz < cp.cz then point.turn = "right" end;
				if np.cz > cp.cz then point.turn = "left" end;
			end;
			--courseplay:debug("--------------------------------------------------------------------", 7);
			--courseplay:debug(string.format("laneDir=%s, point.turn=%s", cp.laneDir, tostring(point.turn)), 7);
			if i == numPoints then
				point.turn = nil;
			end;

			angleDeg = geometry:positiveAngleDeg(angleDeg);

			newLastInLane = geometry:findCrossing(pp, cp, polyPoints, "PFIP");

			if newLastInLane ~= nil then
				--courseplay:debug(string.format("newLastInLane: x=%f, z=%f", newLastInLane.x, newLastInLane.z), 7);
				newLastInLane.angle = point.angle;
				newLastInLane.wait = point.wait;
				newLastInLane.crossing = point.crossing;
				newLastInLane.rev = point.rev;
				newLastInLane.lane = point.lane;
				newLastInLane.laneDir = point.laneDir;
				newLastInLane.lastInLane = true;
				newLastInLane.turn = point.turn;
				newLastInLane.turnStart = courseplay:trueOrNil(i < numPoints);
				newLastInLane.turnEnd = nil;
				newLastInLane.ridgeMarker = 0;
				newLastInLane.generated = true;

				point.wait = nil;
				point.lastInLane = false;
				point.turn = nil;
				point.turnStart = nil;
				point.turnEnd = nil;

			end;
		end; --END cp.lastInLane

		if newFirstInLane ~= nil then
			newFirstInLane.angle = signAngleDeg;
			table.insert(fieldWorkCourse, newFirstInLane);
		end;

		point.angle = signAngleDeg;
		table.insert(fieldWorkCourse, point);

		if newLastInLane ~= nil then
			newLastInLane.angle = signAngleDeg;
			table.insert(fieldWorkCourse, newLastInLane);
		end;

	end; --END for i in numPoints

	if vehicle.cp.startingCorner == 0 then -- start on nearest point to vehicle ; TO BE ADDED TO HUD.
		local cx, _, cz = getWorldTranslation(vehicle.rootNode);
		local starIdx = geometry:getClosestPolyPoint(vehicle.cp.headland.lanes[1], cx, cz);
		startPoint = vehicle.cp.headland.lanes[startIdx];
	else

		startPoint = fieldPoints[1];
	end;

	---############################################################################
	-- (5) ROTATE HEADLAND COURSES
	-------------------------------------------------------------------------------
	courseplay:debug('(5) ROTATE HEADLAND COURSES', 7);
	if vehicle.cp.headland.numLanes > 0 then
		courseplay:debug(string.format('[headlands] rotating headland, number of lanes = %d', numHeadLanes),7);
		if numHeadLanes > 0 then
			-- -------------------------------------------------------------------------
			-- (5.1) ROTATE AND LINK HEADLAND LANES
			if vehicle.cp.headland.orderBefore or turnAround then --each headland lane 's first point is closest to corresponding field begin point or previous lane end point
				local lanes = {};
				local prevLane = {};
				for i,lane in ipairs(vehicle.cp.headland.lanes) do
					local numPoints = #lane;
					courseplay:debug(string.format('curent lane has %d points',numPoints),7);
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
					if i == numHeadLanes and not(turnAround) then -- last headlane link to first fieldWorkCourse point
						courseplay:debug(string.format('last direction is %.2f, field start direction is %.2f',lane[numPoints].angle,fieldWorkCourse[1].angle),7);
						if geometry:getAreAnglesOpposite(lane[numPoints].angle,fieldWorkCourse[1].angle,20,true) then
							local data = lane[numPoints];
							data.cx = (dir == "N" or dir == "S") and lane[numPoints].cx or fieldWorkCourse[1].cx;
							data.cz = (dir == "E" or dir == "W") and lane[numPoints].cz or fieldWorkCourse[1].cz;
							data.cy = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, data.cx, 1, data.cz)
							data.turnStart = true;
							data.turn = orderCW and 'right' or 'left';
							lane[numPoints] = data ;
							fieldWorkCourse[1].turnEnd = true;
						else
							local crossing = geometry:lineIntersection(lane[numPoints-1],lane[numPoints],fieldWorkCourse[1],fieldWorkCourse[2]);
							while crossing.ip1 == 'TIP' or crossing.ip1 == 'NFIP' do
								table.remove(lane);
								numPoints = numPoints - 1;
								crossing = geometry:lineIntersection(lane[numPoints-1],lane[numPoints],fieldWorkCourse[1],fieldWorkCourse[2]);
							end;
							local p1,p2,p3,p4 = lane[numPoints-1],lane[numPoints],fieldWorkCourse[1],fieldWorkCourse[2]
							local dist = Utils.vector2Length(p2.cx-p3.cx,p2.cz-p3.cz);
							if dist > 2 then
								local spline = geometry:smoothSpline(p1, p2, p3, p4, dist/2);
								local data = {};
								for i, point in pairs(spline) do
									if  i < #spline then
										point.angle = geometry:signAngleDeg(point,spline[i+1]);
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
									spline[i] = data;
								end;
								lane = geometry:appendPoly(lane, spline, false, false, prevLane);
							end;
						end;
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
				for i = numHeadLanes, 1, -1 do
					local lane = vehicle.cp.headland.lanes[i];
					local numPoints = #lane;
					local closest = geometry:getClosestPolyPoint(lane, lastFieldworkPoint.cx, lastFieldworkPoint.cz); --TODO: works, but how if lastFieldWorkPoint is nil???
					courseplay:debug(string.format('[after] rotating headland lane=%d, closest=%d -> rotate: numPoints-(closest-1)=%d-(%d-1)=%d', i, closest, numPoints, closest, numPoints - (closest-1)), 7);
					lane = table.rotate(lane, numPoints - (closest - 1));
					if i < numHeadLanes then -- link lanes
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
					if i == numHeadLanes and geometry:getAreAnglesClose(geometry:signAngleDeg(lane[1],lane[2]) + 180,lastFieldworkPoint.angle, 5, 'deg') then
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
		vehicle.maxnumber = #(vehicle.Waypoints);
		vehicle.Waypoints[vehicle.maxnumber].wait = false;

		local p1 = vehicle.Waypoints[vehicle.maxnumber-1];
		local p2 = vehicle.Waypoints[vehicle.maxnumber];
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
	vehicle.maxnumber = #(vehicle.Waypoints)
	if vehicle.maxnumber == 0 then
		courseplay:debug('ERROR: #vehicle.Waypoints == 0 -> cancel and return', 7);
		return;
	end;

	vehicle.recordnumber = 1;
	vehicle.cp.canDrive = true;
	vehicle.Waypoints[1].wait = true;
	vehicle.Waypoints[1].crossing = true;
	vehicle.Waypoints[vehicle.maxnumber].wait = true;
	vehicle.Waypoints[vehicle.maxnumber].crossing = true;
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
	courseplay:debug(string.format("generateCourse() finished: %d lanes, %d headland", numLanes, #(vehicle.cp.headland.lanes)), 7);
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
-- Simplify polygon with Douglas-Peucker

-- local remainingPoints = geometry:simplifyPolygon(points, epsilon);
-- local simplePoly = geometry:newPolyFromIndices(points, remainingPoints); -- make new polygon

