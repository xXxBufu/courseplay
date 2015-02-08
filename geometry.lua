-- parts of the HardOnCollider-project by Matthias Richter on gitHub are used. Original copyright of this project can be found at the end of this file.

local abs, sqrt = math.abs, math.sqrt;

courseplay.geometry = {};

function courseplay.geometry:angleDeg(p1, p2)
	return math.deg(math.atan2(p2.cz-p1.cz, p2.cx-p1.cx));
end;

function courseplay.geometry:appendArc(poly, center, radius, startPoint, endPoint, withFirst, withLast, vehicle) -- TODO: probably needed after use of Douglas Peuker .
	local maxPointDistance = vehicle and vehicle.cp.headland.maxPointDistance or 5;
	local points = {};
	
	radius = math.abs(radius);
	local twoPi = math.pi*2;
	local startAngle = math.atan2(startPoint.cz- center.cz, startPoint.cx - center.cx);
	local endAngle = math.atan2(endPoint.cz - center.cz, endPoint.cx - center.cx);
	if startAngle < 0 then
		startAngle = startAngle + twoPi;
	end;
	if endAngle < 0 then
		endAngle = endAngle + twoPi;
	end;
	local angle;
	if startAngle > endAngle then
		angle = startAngle - endAngle;
	else
		angle = startAngle + twoPi - endAngle;
	end;
	local arcSegmentCount = angle * radius / maxPointDistance;
	local arcAngle = -angle / arcSegmentCount;
	courseplay:debug(string.format('angle is %2.f, %.2f points in arc',math.deg(angle),arcSegmentCount),7);
	if withFirst then table.insert(poly, startPoint) end;
	for i = 1, arcSegmentCount do
		local angle = startAngle + arcAngle * i;
		local point = {
			cx = center.cx + math.cos(angle) * radius,
			cz = center.cz + math.sin(angle) * radius
		};
		table.insert(poly, point);
	end;
	if withLast then table.insert(poly, endPoint) end;
	return poly;
end;

function courseplay.geometry:appendPoly(poly1 ,poly2 ,withStart, withEnd, boundingPline) -- appends poly1 to poly2 with or without start and end points of poly2
	local startidx = withStart and 1 or 2;
	local endidx = withend and #poly2 or #poly2 - 1;
	for idx = startidx, endidx do --add all but first and last point
		if (boundingPline and self:keepPoint(poly2[idx],boundingPline,0)) or not(boundingPline) then
			table.insert(poly1, poly2[idx]);
		end;
	end;
	return poly1;
end;

function courseplay.geometry:clampAngle180(angle)
	while angle > 180 do angle = angle - 360 end;
	while angle < -180 do angle = angle + 360 end;
	return angle;
end;

function courseplay.geometry:cleanPline(pline,boundingPline,offset,vehicle, doRefine)
	doRefine = Utils.getNoNil(deRefine, true);
	local minPointDistance = 0.5;
	local maxPointDistance = 5;
	courseplay:debug('CLEANPLINE',7);
	local newPline, pointsInNewPline = {}, 0;
	table.insert(pline,pline[1]);
	local numPoints = #pline;
	for idx1 = 1, numPoints - 1 do
		local p1 = pline[idx1];
		if p1.cx == p1.cx and p1.cz == p1.cz then --taking away "nan" points
			local idx2 = idx1 + 1;
			--courseplay:debug(string.format('Searching selfintersections after point %d  ' , idx1),7);
			local p2 = pline[idx2];
			if idx2 == numPoints then
				if #newPline > 0 then
					p2 = newPline[1];
				else
					break;
				end;
			end;

			local keep1 = self:keepPoint(p1, boundingPline, offset);
			local keep2 = self:keepPoint(p2, boundingPline, offset);
			if (keep1 and keep2) and Utils.vector2Length(p1.cx-p2.cx,p1.cz-p2.cz) > .1 then
				table.insert(newPline,pline[idx1]);
				pointsInNewPline = pointsInNewPline + 1;
			elseif keep1 or keep2 then --courseplay:debug('Crossing to be found',7);
				for idx3 = 2 , numPoints - 1 do
					local idx4 = idx3 + 1;
					local crossing = self:lineIntersection(p1, p2, pline[idx3], pline[idx4]);
					if (crossing.ip1 == 'TIP' and crossing.ip2 == 'TIP') and crossing.notOnEnds then
						if keep1 then
							table.insert(newPline,p1);
							pointsInNewPline = pointsInNewPline + 1;
							if self:keepPoint(crossing, boundingPline, offset) then
								table.insert(newPline,crossing);
								pointsInNewPline = pointsInNewPline + 1;
							end;
						elseif pointsInNewPline >0 then
							if Utils.vector2Length(newPline[pointsInNewPline].cx-crossing.cx,newPline[pointsInNewPline].cz-crossing.cz) > .1 and self:keepPoint(crossing, boundingPline, offset) then
								table.insert(newPline,crossing);
								pointsInNewPline = pointsInNewPline + 1;
							end;
						elseif self:keepPoint(crossing, boundingPline, offset) then
							table.insert(newPline,crossing);
							pointsInNewPline = pointsInNewPline + 1;
						end;
					end;
				end;
			end;
		end;
	end;
	table.remove(pline);
	if doRefine then
		newPline = self:refinePoly(newPline, maxPointDistance, boundingPline);
	end;
	if self:isSelfCrossing(newPline) then
		newPline = false;
	end;
	return newPline;
end;

-- Douglas-Peucker adapted from http://quangnle.wordpress.com/2012/12/30/corona-sdk-curve-fitting-1-implementation-of-ramer-douglas-peucker-algorithm-to-reduce-points-of-a-curve/
function courseplay.geometry:douglasPeucker(points, firstPointNum, lastPointNum, tolerance, pointIndices)
	local maxDistance = 0;
	local indexFurthest = 0;

	for i=firstPointNum, lastPointNum do
		local distance = self:dPointLine(points[i], points[firstPointNum], points[lastPointNum]);

		if distance > maxDistance then
			maxDistance = distance;
			indexFurthest = i;
		end;
	end;

	if maxDistance > tolerance and indexFurthest ~= 1 then
		table.insert(pointIndices, indexFurthest);

		pointIndices = self:douglasPeucker(points, firstPointNum, indexFurthest, tolerance, pointIndices);
		pointIndices = self:douglasPeucker(points, indexFurthest, lastPointNum, tolerance, pointIndices);
	end;
	return pointIndices;
end;

function courseplay.geometry:dPointLine(point1, point2, point3)
	-- calculates area of the triangle
	local area = abs(0.5 * (point2.cx * point3.cz + point3.cx * 
	point1.cz + point1.cx * point2.cz - point3.cx * point2.cz - point1.cx * 
	point3.cz - point2.cx * point1.cz));

	-- calculates the length of the bottom edge
	local bottom = sqrt((point2.cx - point3.cx) ^ 2 + (point2.cz - point3.cz) ^ 2);

	-- the triangle's height is also the distance found
	local height = area / bottom;

	return height;
end;

function courseplay.geometry:findCrossing(p1,p2, poly, where)
	if where == nil then where = 'TIP' end;
	local numPoints = #poly;
	table.insert(poly, poly[1]);
	local i, found = 0, false;
	local crossing = {};
	while not(found) and i < numPoints do
		i = i + 1;
		crossing = self:lineIntersection(p1, p2, poly[i], poly[i+1]);
		if (crossing.ip1 == where and crossing.ip2 == 'TIP') then
			found = true;
		end;
	end;
	table.remove(poly);
	if found then
	    return crossing;
	else
	    return false;
	end;
end;

function courseplay.geometry:getAreAnglesClose(angle1, angle2, tolerance) 
	if tolerance == nil then tolerance = 0 end;
	angle1 = courseplay.geometry:clampAngle180(angle1);
	angle2 = courseplay.geometry:clampAngle180(angle2);
	return math.abs(angle1 - angle2) <= tolerance;
end;

function courseplay.geometry:getAreAnglesOpposite(angle1, angle2, tolerance)
	if tolerance == nil then tolerance = 0 end;
	angle1 = self:positiveAngleDeg(angle1);
	angle2 = self:positiveAngleDeg(angle2);
	local diff = angle1 < angle2 and angle2 - angle1 or angle1 - angle2;
	diff = 180 - diff; 
	courseplay:debug(string.format('[areOpposite] angle1 = %.2f, angle2 = %.2f -> %s',angle1,angle2,tostring(diff<=tolerance)),7);
	return diff <= tolerance;
end;

function courseplay.geometry:getAreAnglesPerpendicular(angle1,angle2,tolerance) -- TODO: to rewrite not using cos and sin
	if tolerance == nil then tolerance = 0 end;
	if isDeg then
		tolerance = math.rad(tolerance);
		angle1 = math.rad(angle1);
		angle2 = math.rad(angle2);
	end;
	tolerance = math.tan(tolerance);

	if math.abs(math.cos(angle1))-math.abs(math.sin(angle2)) <= tolerance and math.abs(math.sin(angle1))-math.abs(math.cos(angle2)) <= tolerance then
		--courseplay:debug(string.format('a1 = %.4f and a2 = %.4f are perpendicular', angle1 ,angle2 ), 7);
		return true;
	end;
	return false;
end;

function courseplay.geometry:getClosestPolyPoint(poly, x, z)
	local closestDistance = math.huge;
	local closestPointIndex;

	for i, point in pairs(poly) do
		local distanceToPoint = Utils.vector2Length(point.cx-x, point.cz-z);
		if distanceToPoint < closestDistance then
			closestDistance = distanceToPoint;
			closestPointIndex = i;
		end;
	end;
	return closestPointIndex;
end;

function courseplay.geometry:getNormal(p1,p2)
	local length = Utils.vector2Length(p1.cx-p2.cx, p1.cz-p2.cz);
	return {
		nx = (p2.cz-p1.cz) / length,
		nz = (p1.cx-p2.cx) / length
	};
end;

function courseplay.geometry:getPointSide(point,lineP1,lineP2) -- returns the position of 'point' relative to the line defined by 'lineP1' and 'lineP2' watching for P1 in P2 direction.  
	local side = (lineP2.cx-lineP1.cx)*(point.cz-lineP1.cz)-(lineP2.cz-lineP1.cz)*(point.cx-lineP1.cx);
	if side == 0 then return 'on';
	elseif side > 0 then return 'right';
	else return 'left'
	end;
end;

function courseplay.geometry:getPolygonData(poly, px, pz, useC, skipArea, skipDimensions)
	-- This function gets a polygon's area, a boolean if x,z is inside the polygon, the poly's dimensions and the poly's direction (clockwise vs. counter-clockwise).
	-- Since all of those queries require a for loop through the polygon's vertices, it is better to combine them into one big query.

	if useC == nil then useC = true; end;
	local x,z = useC and 'cx' or 'x', useC and 'cz' or 'z';
	local numPoints = #poly;
	local cp,np,pp;
	local fp = poly[1];

	-- POINT IN POLYGON (Jordan method) -- @src: http://de.wikipedia.org/wiki/Punkt-in-Polygon-Test_nach_Jordan
	-- returns:
	--	 1	point is inside of poly
	--	-1	point is outside of poly
	--	 0	point is directly on poly
	local getPointInPoly = px ~= nil and pz ~= nil;
	local pointInPoly = -1;
	local point = { [x] = px, [z] = pz };

	-- AREA -- @src: https://gist.github.com/listochkin/1200393
	-- area will be twice the signed area of the polygon. If the poly is counter-clockwise, the area will be positive. If clockwise, the area will be negative.
	-- returns: real area (|area| / 2)
	local area = 0;

	-- DIMENSIONS
	local dimensions = {
		minX =  999999,
		maxX = -999999,
		minZ =  999999,
		maxZ = -999999
	};

	--[[
	-- DIRECTION
	-- offset test points
	local dirX,dirZ = self:getPointDirection(poly[1], poly[2], useC);
	local offsetRight = {
		[x] = poly[2][x] - dirZ,
		[z] = poly[2][z] + dirX,
		isInPoly = false
	};
	local offsetLeft = {
		[x] = poly[2][x] + dirZ,
		[z] = poly[2][z] - dirX,
		isInPoly = false
	};
	-- clockwise vs counterclockwise variables
	local dirArea, dirSuccess, dirTries = 0, false, 1;
	]]

	-- ############################################################

	for i=1, numPoints do
		cp = poly[i];
		np = i < numPoints and poly[i+1] or poly[1];
		pp = i > 1 and poly[i-1] or poly[numPoints];

		-- point in polygon
		if getPointInPoly and pointInPoly ~= 0 then
			pointInPoly = pointInPoly * courseplay.utils:crossProductQuery(point, cp, np, useC);
		end;

		-- area
		if not skipArea then
			area = area + cp[x] * np[z];
			area = area - cp[z] * np[x];
		end;

		-- dimensions
		if not skipDimensions then
			if cp[x] < dimensions.minX then dimensions.minX = cp[x]; end;
			if cp[x] > dimensions.maxX then dimensions.maxX = cp[x]; end;
			if cp[z] < dimensions.minZ then dimensions.minZ = cp[z]; end;
			if cp[z] > dimensions.maxZ then dimensions.maxZ = cp[z]; end;
		end;

		--[[
		-- direction
		if i < numPoints then
			local pointStart = {
				[x] = cp[x] - fp[x];
				[z] = cp[z] - fp[z];
			};
			local pointEnd = {
				[x] = np[x] - fp[x];
				[z] = np[z] - fp[z];
			};
			dirArea = dirArea + (pointStart[x] * -pointEnd[z]) - (pointEnd[x] * -pointStart[z]);
		end;

		-- offset right point in poly
		if ((cp[z] > offsetRight[z]) ~= (pp[z] > offsetRight[z])) and (offsetRight[x] < (pp[x] - cp[x]) * (offsetRight[z] - cp[z]) / (pp[z] - cp[z]) + cp[x]) then
			offsetRight.isInPoly = not offsetRight.isInPoly;
		end;

		-- offset left point in poly
		if ((cp[z] > offsetLeft[z])  ~= (pp[z] > offsetLeft[z]))  and (offsetLeft[x]  < (pp[x] - cp[x]) * (offsetLeft[z]  - cp[z]) / (pp[z] - cp[z]) + cp[x]) then
			offsetLeft.isInPoly = not offsetLeft.isInPoly;
		end;
		]]
	end;

	if getPointInPoly then
		pointInPoly = pointInPoly ~= -1;
	else
		pointInPoly = nil;
	end;

	if not skipDimensions then
		dimensions.width  = dimensions.maxX - dimensions.minX;
		dimensions.height = dimensions.maxZ - dimensions.minZ;
	else
		dimensions = nil;
	end;

	local isClockwise;
	if not skipArea then
		isClockwise = area < 0;
		area = math.abs(area) / 2;
	else
		area = nil;
		isClockwise = nil;
	end;

	return area, pointInPoly, dimensions, isClockwise;
end;

function courseplay.geometry:getPointDirection(cp, np, useC)
	if useC == nil then useC = true; end;
	local x,z = 'x','z';
	if useC then
		x,z = 'cx','cz';
	end;

	local dx, dz = np[x] - cp[x], np[z] - cp[z];
	local vl = Utils.vector2Length(dx, dz);
	if vl and vl > 0.0001 then
		dx = dx / vl;
		dz = dz / vl;
	end;
	return dx, dz, vl;
end;

function courseplay.geometry:getRelativePointDirection(pp, cp, np, useC)
	if pp == nil or cp == nil or np == nil then return nil; end;
	if useC == nil then useC = true; end;

	local dx1, dz1 = self:getPointDirection(pp, cp, useC);
	local dx2, dz2 = self:getPointDirection(cp, np, useC);

	local rot1 = Utils.getYRotationFromDirection(dx1, dz1);
	local rot2 = Utils.getYRotationFromDirection(dx2, dz2);

	local rotDelta = rot1 - rot2; --TODO: rot2 - rot1 ?
	
	return Utils.getDirectionFromYRotation(rotDelta);
end;

function courseplay.geometry:invertAngleDeg(ang)
	if ang > 0 then
		return ang - 180;
	else
		return ang + 180;
	end;
end;

function courseplay.geometry:isSelfCrossing(poly)
	local crossingFound = false;
	local idx1 = 1;
	local numPoints = #poly;
	while not crossingFound and idx1 <= numPoints - 3 do
		local p1 = poly[idx1];
		local p2 = poly[idx1 + 1];
		local idx2 = idx1 + 2;
		while not crossingFound and idx2 <= numPoints - 1 do
			local p3 = poly[idx2];
			local p4 = poly[idx2 + 1];
			local crossing = self:lineIntersection(p1,p2,p3,p4);
			local dist = self:pointDistToLine(p3,p1,p2);
			--courseplay:debug(string.format('dist = %.2f',dist),7);
			crossingFound = (crossing.ip1 == 'TIP' and crossing.ip2 == 'TIP') or dist == 0;
			idx2 = idx2 + 1;
		end;
		idx1 = idx1 + 1;
	end;
	return crossingFound;
end; 

function courseplay.geometry:keepPoint(point, pline, offset)
	local dist, inPline = self:minDistToPline(point, pline);
	local keep = inPline and (courseplay:round(dist,5) >= courseplay:round(math.abs(offset),5)) ;
	courseplay:debug(string.format(' dist = %.8f offset = %.8f keep = %s ', courseplay:round(dist,5),courseplay:round(offset,5), tostring(keep)),7);
	return keep;
end;

-- @src: http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect#comment19248344_1968345
function courseplay.geometry:lineIntersection(p1, p2, p3, p4)
--	courseplay:debug(string.format('/t/t%.4f,%.4f - %.4f,%.4f ',p1.cx,p1.cz,p2.cx,p2.cz),7);
--	courseplay:debug(string.format('/t/t%.4f,%.4f - %.4f,%.4f ',p3.cx,p3.cz,p4.cx,p4.cz),7);
	local s1_x, s1_y, s2_x, s2_y, x, z;
	s1_x = p2.cx - p1.cx;
	s1_y = p2.cz - p1.cz;
	s2_x = p4.cx - p3.cx;
	s2_y = p4.cz - p3.cz;

	local denom = (-s2_x * s1_y + s1_x * s2_y);
	local x, z, pos1, pos2, onEnd, collinear = 0, 0,'NO', 'NO', false, false;
	
	if denom ~= 0 then
		local s = (-s1_y * (p1.cx - p3.cx) + s1_x * (p1.cz - p3.cz)) / denom; -- on p3-p4
		local t = ( s2_x * (p1.cz - p3.cz) - s2_y * (p1.cx - p3.cx)) / denom; -- on p1-p2

		x = p1.cx + (t * s1_x);
		z = p1.cz + (t * s1_y);
		onEnd = s==1 or s==0 or t==0 or t==1;
		if (s >= 0 and s <= 1) then
			pos2 = 'TIP';
		elseif s > 1 then
			pos2 = 'PFIP';
		else
			pos2 = 'NFIP';
		end;
		if (t >= 0 and t <= 1) then
			pos1 ='TIP';
		elseif t > 1 then
			pos1 = 'PFIP';
		else
			pos1 = 'NFIP';
		end;
	end;
	--courseplay:debug(string.format('lineCrossing -> %.2f, %.2f - %s - %s',x,z,pos1,pos2),7);
	return {
		cx = x,
		cz = z,
		ip1 = pos1,
		ip2 = pos2,
		notOnEnds = not(onEnd),
	};
end;

function courseplay.geometry:lineIntersectsPoly(point1, point2, poly) -- TODO: not used anywhere
	for k,wp in pairs(poly.points) do
		local nextPointIdx;
		if k < poly.numPoints then
			nextPointIdx = k + 1;
		elseif k == poly.numPoints then
			nextPointIdx = 1;
		end;
		local nextPoint = poly.points[nextPointIdx];

		local intersects = self:segmentsIntersection(point1.x, point1.z, point2.x, point2.z, wp.cx, wp.cz, nextPoint.cx, nextPoint.cz);
		if intersects then
			return intersects;
		end;
	end;
	return nil;
end;

function courseplay.geometry:minDistToPline(point, pline)
	local numPoints = #pline;
	table.insert(pline,pline[1]);
	local pointInPline = -1;
	local minDist = math.huge;
	local extPoint = {
		cx = 999999,
		cz = 999999
	};
	for i=1, numPoints do
		local cp = pline[i];
		local np = pline[i+1];
		local crossing = courseplay.geometry:lineIntersection(extPoint,point,cp,np); -- TODO: move to geometry
		if crossing.ip1 == 'TIP' and crossing.ip2 == 'TIP' then  pointInPline = pointInPline * -1 end;
		local dist, _ = self:pointDistToLine(point,cp,np);
		if dist < minDist then
			minDist = dist;
		end;
	end;
	pointInPline = pointInPline == 1 ;
	courseplay:debug(string.format('Point at %.4f InPoly = %s', minDist, tostring(pointInPline)),7);
	table.remove(pline);
	return minDist, pointInPline;
end;

function courseplay.geometry:mirrorPoint(p1, p2, useC)
	--returns p1 mirrored at p2
	if useC == nil then useC = true;  end;
	local x, z = 'x', 'z';
	if useC then
		x, z = 'cx', 'cz';
	end;
	local sp = {
		[x] = p2[x] + (p2[x] - p1[x]),
		[z] = p2[z] + (p2[z] - p1[z])
	} ;
	return sp;
end;

function courseplay.geometry:newPolyFromIndices(polyIn, indices)
	local polyOut = {};
	for _,index in pairs(indices) do
		local point = { cx = polyIn[index].cx, cz = polyIn[index].cz };
		table.insert(polyOut, point);
	end;
	polyOut = self:setPolyCounterClockwise(polyOut);
	return polyOut;
end;

function courseplay.geometry:offsetPoint(point, normal, offset)
	local x = point.cx + (normal.nx * offset);
	local z = point.cz + (normal.nz * offset);
	--courseplay:debug(string.format(' point %.4f, %.4f offset to %.4f, %.4f', point.cx,point.cz,x,z),7);
	return {
		cx = x,
		cz = z
	};
end;

function courseplay.geometry:offsetPoly(pline, offset, vehicle, doRefine)
	local pline1 = self:untrimmedOffsetPline(pline, offset, vehicle);
	pline1 = self:cleanPline(pline1, pline, offset, vehicle, doRefine);
	return pline1;
end;

function courseplay.geometry:pointDistToLine(point,linePoint1,linePoint2)
	local segLength = Utils.vector2Length(linePoint1.cx-linePoint2.cx,linePoint1.cz-linePoint2.cz);
	local dist;
	local t = math.huge;
	if segLenth == 0 then
		dist = Utils.vector2Length(linePoint1.cx - point.cx,linePoint1.cz - point.cz);
	else
		t = ((point.cx - linePoint1.cx) * (linePoint2.cx - linePoint1.cx) + (point.cz - linePoint1.cz) * (linePoint2.cz - linePoint1.cz) ) / (segLength * segLength);
		if t < 0 then
			dist = Utils.vector2Length(linePoint1.cx - point.cx,linePoint1.cz - point.cz);
		elseif t > 1 then
			dist = Utils.vector2Length(linePoint2.cx - point.cx,linePoint2.cz - point.cz);
		else
			local x = linePoint1.cx + t * (linePoint2.cx - linePoint1.cx);
			local z = linePoint1.cz + t * (linePoint2.cz - linePoint1.cz);
			dist = Utils.vector2Length(point.cx - x, point.cz - z);
		end;
	end;
	return dist, (t >= 0 and t <= 1);
end;

function courseplay.geometry:positiveAngleDeg(ang) 
	while ang < 0 do
		ang = ang + 360;
	end;
	return ang;
end;

function courseplay.geometry:refinePoly(poly, maxDistance, boundingPline)
	local pointsInPline = #poly;
	local newPline = {};
	for idx = 1, pointsInPline do
		local idxPrev = idx - 1 ;
		if idx == 1 then
			idxPrev = pointsInPline;
		end;
		local idxNext = idx + 1 ;
		if idx == pointsInPline then
			idxNext = 1;
		end;
		local idxNext2 = idxNext + 1 ;
		if idxNext == pointsInPline then
			idxNext2 = 1;
		end;
		local p1, p2, p3, p4 = poly[idxPrev], poly[idx], poly[idxNext], poly[idxNext2];
		if Utils.vector2Length(p2.cx - p3.cx, p2.cz - p3.cz) > maxDistance then
			local spline = self:smoothSpline(p1, p2, p3, p4, Utils.vector2Length(p2.cx - p3.cx, p2.cz - p3.cz)/maxDistance);
			newPline = self:appendPoly(newPline, spline, true, false, boundingPline);
		else
			table.insert(newPline,p2);
		end;
	end;
	return newPline;
end;

function courseplay.geometry:samePoints(p1,p2) 
	return p1.cx == p2.cx and p1.cz == p2.cz;
end;

-- @src: http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect#comment19248344_1968345
function courseplay.geometry:segmentsIntersection(A1x, A1y, A2x, A2y, B1x, B1y, B2x, B2y, where) -- TODO: only used in lineIntersectsPoly, which itself isn't used anywhere
	if where == nil then
		where = 'segment';
	end;

	local s1_x = A2x - A1x;
	local s1_y = A2y - A1y;
	local s2_x = B2x - B1x;
	local s2_y = B2y - B1y;

	local denom = (-s2_x * s1_y + s1_x * s2_y);
	if math.abs(denom) > 0 then
		local s = (-s1_y * (A1x - B1x) + s1_x * (A1y - B1y)) / denom; --concerns p3,p4
		local t = ( s2_x * (A1y - B1y) - s2_y * (A1x - B1x)) / denom; --concerns p1,p2
		if ((s >= 0 and s <= 1 and t >= 0 and t <= 1) and where == 'segment')
			or (s <= 0 and t >= 0 and where == 'between')
			or (s >= 0 and s <= 1 and t >= 0 and where == 'lineonsegment') then
			--Collision detected
			--courseplay:debug(string.format('segInter ( %s, denom = %.4f, s = %.4f, t = %.4f )',where, denom, s, t), 7);
			local x = A1x + (t * s1_x);
			local z = A1y + (t * s1_y);
			return { x = x, z = z };
		end;
	end;
	--No collision
	return nil;
end;

function courseplay.geometry:setPolyClockwise(poly)
	local _, _, _, isClockwise = self:getPolygonData(poly, nil, nil, true, false, true); -- TODO: move that fn to geometry
	if isClockwise then
		return poly;
	else
		courseplay:debug(string.format('reversing order of polygon, isClockwise %s', tostring(isClockwise)),7);
		local newPoly = table.reverse(poly);
		return newPoly;
	end;
end;

function courseplay.geometry:setPolyCounterClockwise(poly)
	local _, _, _, isClockwise = self:getPolygonData(poly, nil, nil, true, false, true); -- TODO: move that fn to geometry
	if isClockwise then
		courseplay:debug('reversing order of polygon',7);
		local newPoly = table.reverse(poly);
		return newPoly;
	else
		return poly;
	end;
end;

function courseplay.geometry:signAngleDeg(p1,p2)
	return math.deg(math.atan2(p2.cx-p1.cx, p2.cz-p1.cz));
end;

function courseplay.geometry:simplifyPolygon(points, epsilon)
	local remainingPoints = {};

	remainingPoints = self:douglasPeucker(points, 1, #points, epsilon, remainingPoints);
	table.sort(remainingPoints);

	local distance1 = self:dPointLine(points[1], points[remainingPoints[1]], points[ remainingPoints[#remainingPoints] ]);
	local distance2 = self:dPointLine(points[#points], points[remainingPoints[1]], points[ remainingPoints[#remainingPoints] ]);
	if distance1 > epsilon or distance2 > epsilon then -- we need at least one of the end points
		local distance3 = self:dPointLine(points[#points],points[1], points[ remainingPoints[#remainingPoints] ]);
		local distance4 = self:dPointLine(points[1],points[#points], points[ remainingPoints[1] ]);
		if distance3 < distance4 then
			table.insert(remainingPoints, 1, 1);
		else
			table.insert(remainingPoints, #points);
		end;
	end;

	return remainingPoints;
end;

-- @src: http://www.efg2.com/Lab/Graphics/Jean-YvesQueinecBezierCurves.htm
function courseplay.geometry:smoothSpline(refPoint1, startPoint, endPoint , refPoint2 , steps, useC, addHeight, useCrossing)
	if useCrossing == nil then useCrossing = true; end;
	if useC == nil then useC = true; end;
	if addHeight == nil then addHeight = false; end;

	local steps = steps or 5;
	local spline = {};
	local x,y,z = 'x','y','z';
	if useC then
		x,y,z = 'cx','cy','cz';
	end;
	if refPoint1 or RefPoint2 then
		local p1,p2,p3,p4;
		p1 = startPoint;
		p4 = endPoint;
		if refPoint1 and refPoint2 then
			local crossingPoint = self:lineIntersection(refPoint1, startPoint, endPoint, refPoint2);
			if crossingPoint.ip1 == 'PFIP' and crossingPoint.ip2 == 'NFIP' and useCrossing then -- crossing point is used as reference
				crossingPoint.x, crossingPoint.z = crossingPoint.cx, crossingPoint.cz;
				courseplay:debug(string.format('lines are crossing at : %.4f, %.4f', crossingPoint[x], crossingPoint[z]), 7);
				p2 = crossingPoint;
				p3 = crossingPoint;
			else
				p2 = self:mirrorPoint(refPoint1, startPoint, useC);
				p3 = self:mirrorPoint(refPoint2, endPoint, useC);
			end;
		else
			if refPoint1 then
				p3 = self:mirrorPoint(refPoint1, startPoint, useC);
				p2 = p3;
			else
				p2 = self:mirrorPoint(refPoint2, endPoint, useC);
				p3 = p2;
			end;
		end;
		for t = 0, 1, (1/steps) do
			local point = {
				[x] = math.pow(1-t, 3) * p1[x] + 3 * math.pow(1-t, 2) * t * p2[x] + 3 * (1-t) * t*t *  p3[x] + t*t*t * p4[x],
				[z] = math.pow(1-t, 3) * p1[z] + 3 * math.pow(1-t, 2) * t * p2[z] + 3 * (1-t) * t*t *  p3[z] + t*t*t * p4[z]
			};
			point[y] = addHeight and getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, point[x], 1, point[z]) + 3 or nil;

			table.insert(spline, point);
			courseplay:debug(string.format('smoothSpline adding point : %.2f, %.2f', point[x], point[z]), 7);
		end;
	else
		table.insert(spline,startPoint);
		table.insert(spline,endPoint);
	end;
	return spline;
end;

function courseplay.geometry:untrimmedOffsetPline(pline, offset, vehicle)
	local maxPointDistance = vehicle and vehicle.cp.headland.maxPointDistance or 5;
	table.insert(pline,pline[1]);
	local numPoints = #pline;
	local offPline = {};
	for idx1 = 1, numPoints do
		local idx2 = idx1 == numPoints and 1 or idx1 + 1;
		local idx3 = idx2 == numPoints and 1 or idx2 + 1;
		local normal = self:getNormal(pline[idx1], pline[idx2]);
		local s1p1 = self:offsetPoint(pline[idx1], normal, offset);
		local s1p2 = self:offsetPoint(pline[idx2], normal, offset);
		normal = self:getNormal(pline[idx2], pline[idx3]);
		local s2p1 = self:offsetPoint(pline[idx2], normal, offset);
		local s2p2 = self:offsetPoint(pline[idx3], normal, offset);
		local crossing = self:lineIntersection(s1p1, s1p2, s2p1, s2p2);
		if idx1 == numPoints - 1 then
			table.insert(offPline, s1p2);
		elseif crossing.ip1 == 'TIP' and crossing.ip2 == 'TIP' then
			table.insert(offPline, crossing);
		elseif crossing.ip1 == 'PFIP' and crossing.ip2 == 'NFIP' then
			if Utils.vector2Length(s1p1.cx-crossing.cx,s1p1.cz-crossing.cz) > maxPointDistance then
				table.insert(offPline, s1p2);
			end;
			if Utils.vector2Length(s1p2.cx-s2p1.cx,s1p2.cz-s2p1.cz) > maxPointDistance then
				offPline = self:appendArc(offPline,pline[idx2],offset,s1p2,s2p1,false, false, vehicle)
			else
				table.insert(offPline, crossing);
			end;
			if Utils.vector2Length(s2p2.cx-crossing.cx,s2p2.cz-crossing.cz) > maxPointDistance then
				table.insert(offPline, s2p1);
			end;
		else
			table.insert(offPline, s1p2);
			table.insert(offPline, s2p1);
		end;
	end;
	courseplay:debug(string.format('Untrimmed offset finished with %d points', #offPline),7);
	table.remove(pline);
	return offPline;
end;

function courseplay.geometry:getInnerPoly(poly,vehicle)
	if vehicle.cp.headland.numLanes > 0 then
		local offset = (vehicle.cp.workWidth * vehicle.cp.headland.numLanes + .5) * -1;
		poly = self:offsetPoly(poly, offset, vehicle, false);
	end;
	return poly;
end;

--
-- cpScanner by upsidedown, ported to coursePlay by fck54
-- test mod for courseplay scanner code
--
-- author: upsidedown
-- initial version: 24.11.2013
-- start of version for edge simplification: 1.1.2015
-- parts of the HardOnCollider-project by Matthias Richter on gitHub are used. Original copyright of this project can be found at the end of this file.
-- Douglas-Peuker code was taken from http://quangnle.wordpress.com/2012/12/30/corona-sdk-curve-fitting-1-implementation-of-ramer-douglas-peucker-algorithm-to-reduce-points-of-a-curve/

function courseplay.geometry:addPolyPoints(poly,x,z,...)
	if x ~= nil and z ~= nil then
		table.insert(poly,{cx = x, cz=z})
		courseplay.geometry:addPolyPoints(poly,...)
	end;
end;

function courseplay.geometry:newPolygon(...)
	local new = {}
	courseplay.geometry:addPolyPoints(new,...)	
	return new
end;

-- here is the master piece ---------------------------------------------------------------------
function courseplay.geometry:genWorkCourse(points, vehicle, startPoint)
	
	local width = vehicle.cp.workWidth;
	local workDirection = vehicle.cp.startingDirection or courseplay:currentVehAngle(vehicle); --passed in degre
	local startPoint = vehicle.cp.startingPoint or startPoint;
	local turningTime = 35 * (2 - vehicle.cp.directionVariance); --s
	local hoppingTime = 3*90 * (2 - vehicle.cp.directionVariance); --s‏
	local speed = 15/3.6; --m/s!

	--simplify:
	local Epsilon = Utils.clamp(vehicle.cp.fieldEdge.douglasPeuckerEpsilon, .5 , width/2);
	local remainingPoints = courseplay.geometry:simplifyPolygon(points,Epsilon)
	local simplePoly = courseplay.geometry:newPolyFromIndices(points,remainingPoints)
	
	local isConvex = courseplay.geometry:isConvex(simplePoly)
	courseplay:debug("isConvex: "..tostring(isConvex),7)
	
	courseplay:debug("area: "..tostring(courseplay.geometry:getArea(simplePoly)),7)
	local area = courseplay.geometry:getArea(simplePoly);
	hoppingTime = hoppingTime*math.sqrt(area/10000); 
	
	local split = {};

	local doAutoCourse = false;
	courseplay:debug('Vehicle angle is ' .. tostring(workDirection),7)
	local manualAngle = math.rad(workDirection); -- Using vehicle direction for testing ... 
	

	if doAutoCourse then
		split = self:splitConvex(simplePoly)
		courseplay:debug("split into: "..tostring(#split),7)
	end;
	courseplay:debug("split into: "..tostring(#split),7)

	local groupPolys, groupAngles; 
	if doAutoCourse then
		courseplay:debug("calling optimization:",7)
		groupPolys, groupAngles = courseplay.geometry:getOptimalCourse(split, speed, width, turningTime,hoppingTime)
	else
		courseplay:debug("angle is set manually to "..tostring(math.deg(manualAngle)),7)
		groupPolys = {simplePoly}
		groupAngles = {manualAngle};
	end;

	local workLines = {};
	for variant = 1,4 do -- why using variant 1 to 4 ? 
		courseplay:debug("calculating courses for variant: "..tostring(variant),7)
		workLines[variant] = courseplay.geometry:fillSubFieldCourses(groupPolys,width, groupAngles, variant)
		for sub,workLines in pairs(workLines[variant]) do
			courseplay:debug("checking field leaving at plot "..tostring(sub),7)
			courseplay.geometry:addInFieldPoints(workLines,simplePoly)
		end;
	end;
	local finalCourse = courseplay.geometry:mergeFinalCourse(workLines,startPoint);
	
	finalCourse = courseplay.geometry:addInFieldPoints(finalCourse,simplePoly)
	
	return courseplay.geometry:course2CPwaypoints(finalCourse);
end;

function courseplay.geometry:copyNoColl(poly)
	local res = {};
	
	local function areCollinear(p, q, r, eps)
		return math.abs(courseplay.geometry:det(q.cx-p.cx, q.cz-p.cz,  r.cx-p.cx,r.cz-p.cz)) <= (eps or 1e-32)
	end
	
	for k,point in pairs(poly) do
		table.insert(res,point)
	end;
	
	for k=#poly-1,2,-1 do
		if areCollinear(res[k+1],res[k],res[k-1],0.001) then
			table.remove(res,k)
		end;
	end;
	courseplay:debug("reduced polygon from "..tostring(#poly).." to "..tostring(#res).." points",7)
	return res;
end;

function courseplay.geometry:shrinkIntersection(res,ind1,delta)
	local ind2 = ind1+1;
	if res[ind2] > res[ind1] then
		res[ind2] = res[ind2]-delta;
		res[ind1] = res[ind1]+delta;		
	else
		res[ind2] = res[ind2]+delta;
		res[ind1] = res[ind1]-delta;		
	end;	
end;

function courseplay.geometry:getAngle(vec1,vec2)	
	local L1 = math.sqrt(vec1.cx^2+vec1.cz^2);
	local L2 = math.sqrt(vec2.cx^2+vec2.cz^2);
	local cosAlpha = (vec1.cx*vec2.cx + vec1.cz*vec2.cz)/(L1*L2);
	local alpha = math.acos(cosAlpha)
	
	return alpha, cosAlpha;	
end;

function courseplay.geometry:norm(x,z)
	local L = math.sqrt(x^2+z^2);
	return x/L, z/L
end;

function courseplay.geometry:shrinkPolygon(poly,paraDis)
	local new = {};
	local last_k = #poly;
	local k = 1;
	paraDis = 2*paraDis; --whyever, gives right results...
	while k<=#poly do
		local next_k = k+1;
		if k == #poly then
			next_k = 1;
		end;
		
		local dx1, dz1 = courseplay.geometry:norm(poly[last_k].cx-poly[k].cx,poly[last_k].cz-poly[k].cz);
		local dx2, dz2 = courseplay.geometry:norm(poly[next_k].cx-poly[k].cx,poly[next_k].cz-poly[k].cz);
			
		
		local v1 = {cx = dx1, cz = dz1}
		local v2 = {cx = dx2, cz = dz2}
		
		local alpha = courseplay.geometry:getAngle(v1,v2)
		local ccw = courseplay.geometry:ccw(poly[last_k], poly[k], poly[next_k],1.0);
		
		
		local r = 0;
		local direc= 1.0;
		if ccw then
			r = paraDis/math.sin(alpha);
		else
			r = paraDis/math.cos(math.pi/2-alpha);
			direc = -1.0;
		end;
		-- direc = -direc;
		
		local newX = poly[k].cx + 0.5*direc*(dx1+dx2)*r;
		local newZ = poly[k].cz + 0.5*direc*(dz1+dz2)*r;
		
		new[k] = {cx=newX, cz=newZ};
		-- print("alpha: "..tostring(alpha*180/math.pi).." "..tostring(ccw).." r="..tostring(r))
		
		last_k = k;
		k = k+1;
	end;
	
	
	local k = 1;
	while true do --search for intersections and remove them
		local found, foundIndex, x, z = courseplay.geometry:searchNextIntersection(new, k)
		if found then
			local removeIndices = {};
			local rem = k+1;
			while rem~=foundIndex  do
				table.insert(removeIndices,rem);
				rem = rem+1;
				if rem > #new then
					rem = 1;
				end;
			end;
			table.sort(removeIndices);--we have to remove from the back or we get shifting
			while #removeIndices>0 do
				table.remove(new,removeIndices[#removeIndices]);
				table.remove(removeIndices)			
			end;
			if x~= nil and z~= nil then
				table.insert(new,k+1,{cx=x,cz=z})
			end;			
		else		
			k = k+1;
		end;
		if k > #new then
			break;
		end;
	end;
	if courseplay.geometry:getArea(new) < 0 then
		courseplay:debug("negative ara in shrinking! ",courseplay.geometry:getArea(new),7)
		new = {};
	end;
	-- print(tostring(#new).." points, area: "..tostring(courseplay.geometry:getArea(new)))	
	return new
end;

function courseplay.geometry:searchNextIntersection(poly, startIndex)
	local found = false;
	
	local startIndex2 = startIndex+1;
	if startIndex == #poly then
		startIndex2 = 1;
	end;
	
	local last_k = #poly;
	local k = 1;
	while k <= #poly do
		if k == startIndex then
			last_k = k;
			k = k+1;
		else
			if courseplay.geometry:segmentsInterset(poly[startIndex],poly[startIndex2], poly[k],poly[last_k]) then
				-- print("found intersection: "..tostring(startIndex).." "..tostring(k))
				--calc intersection point:
				local dx = poly[last_k].cx-poly[k].cx;
				local dz = poly[last_k].cz-poly[k].cz;
				local res = courseplay.geometry:intersectionsWithRay({poly[startIndex],poly[startIndex2]},poly[k].cx,poly[k].cz, dx,dz);
				-- print(tostring(res).." "..tostring(res[1]))
				local x,z
				-- for _,value in pairs(res) do
					-- print(value)
				-- end;
				-- print("))))")
				if #res > 1 then
					-- for k,value in pairs(res) do
						-- if not(value >= 0 and value <= 1) then
							-- -- x = poly[k].cx + value*dx;
							-- -- z = poly[k].cz + value*dz;
							-- -- break;
							-- print("value ",value)
						-- end;
					-- end;
					x = poly[k].cx + res[1]*dx;
					z = poly[k].cz + res[1]*dz;
				else
					courseplay:debug("segment intersection returns ambiguous result",7)
				end;
				return true, k, x, z
			else
				last_k = k;
				k = k+1;
			end;
		end;		
	end;
	
	
	return false, nil
end;

function courseplay.geometry:attachRoundCourse(waypoints,shrinks,attachStart,doCCW)
	local wpChains = {};
	for k,shrink in pairs(shrinks) do
		local wpChain = {};
		local last_kk = #shrink;
		local kk = 1;
		while kk <= #shrink do
		
			local dx = -shrink[last_kk].cx+shrink[kk].cx;
			local dz = -shrink[last_kk].cz+shrink[kk].cz;
			local dist = math.sqrt(dx^2 + dz^2);
			local N = math.floor(dist/5);
			local step = dist/N;
			
			dx = step*dx/dist;
			dz = step*dz/dist;
			
			for kk = 1,N do
				local bWp = {};
				bWp.cx = shrink[last_kk].cx + kk*dx;
				bWp.cz = shrink[last_kk].cz + kk*dz;
				bWp.speed = 20; --need speed!
				if doCCW then
					table.insert(wpChain,1,bWp);
				else
					table.insert(wpChain,bWp);
				end;
			end;
			
			local wp = {};
			wp.cx = shrink[kk].cx;
			wp.cz = shrink[kk].cz;
			wp.speed = 20;
			if doCCW then
				table.insert(wpChain,1,wp);
			else
				table.insert(wpChain,wp);
			end;
		
			
			last_kk = kk;
			kk = kk+1;
		end;
		-- if (doCCW and not attachStart) or (not doCCW and attachStart) then
		if not attachStart then
			table.insert(wpChains,wpChain);
		else
			table.insert(wpChains,1,wpChain);		
		end;
	end;
	--now we have all the points. we only need to distribute them...
	
	-- local previousLine = waypoints;
	local previousLine = {};
	if attachStart then
		previousLine[1] = waypoints[2];
		previousLine[2] = waypoints[1];		
	else
		previousLine[1] = waypoints[#waypoints-1];
		previousLine[2] = waypoints[#waypoints];		
	end;
	
	
	-- local refIndex1 = 1;
	-- local refIndex2 = 2;
	
	-- if not attachStart then
		-- refIndex1 = #waypoints;
		-- refIndex2 = #waypoints-1;
	-- end;
	
	-- local startX = waypoints[refIndex1].cx;
	-- local startZ = waypoints[refIndex1].cz;
	local wpAdd = {};
	for k=#wpChains,1,-1 do
		local distance2Min = math.huge;
		local nearestIndex = 0;
		for kk,point in pairs(wpChains[k]) do
			local distance2 = (point.cx-previousLine[#previousLine].cx)^2 + (point.cz-previousLine[#previousLine].cz)^2;
			if distance2<distance2Min then
				-- if courseplay.geometry:ccw(waypoints[refIndex2],waypoints[refIndex1],point) == doCCW then
				----- IF COS_alpha in triangle #last-1,#last,point < 0 then ---WILL GIVE SMOOTHER TRANSITIONS
					distance2Min = distance2;
					nearestIndex = kk;
				-- end;
			end;
		end;
		for cnt = 1,nearestIndex-1,1 do --shift nearest point to front
			table.insert(wpChains[k],wpChains[k][1])
			table.remove(wpChains[k],1)
		end;
		for cnt=1,#wpChains[k] do
			table.insert(wpAdd,wpChains[k][cnt])
		end;
		previousLine = wpChains[k];
	end;
	
	for k, point in pairs(wpAdd) do
		if attachStart then
			table.insert(waypoints,k,point)
		else
			table.insert(waypoints,point)
		end;
	end;
		
	waypoints[1].wait = true;
	waypoints[#waypoints].wait = true;	
	
	return wpAdd;
end;

function courseplay.geometry:course2CPwaypoints(finalCourse)
	local wayPoints = {};
	local lastWorkStart = false;
	local lastTurning = false;
	
	local wp = {};
	local numPoints = #finalCourse;

	for k,point in pairs(finalCourse) do
		wp.cx = point.cx;
		wp.cz = point.cz;
		wp.ridgeMarker = 0;
		wp.generated = true;
		
		if point.workstart then
			lastWorkStart = true;
		elseif lastWorkStart then
			lastWorkStart = false;
			wp.angle = self:signAngleDeg(finalCourse[k-1],point);
			local dx = -finalCourse[k-1].cx+finalCourse[k].cx;
			local dz = -finalCourse[k-1].cz+finalCourse[k].cz;
			local dist = math.sqrt(dx^2 + dz^2);
			local N = math.floor(dist/5);
			local step = dist/N;
			
			dx = step*dx/dist;
			dz = step*dz/dist;
			
			for kk = 0,N do
				local bWp = {};
				bWp.cx = finalCourse[k-1].cx + kk*dx;
				bWp.cz = finalCourse[k-1].cz + kk*dz;
				bWp.turnEnd = false;
				if kk == 0 and k > 1 then
					bWp.turnEnd = true;
				end;
				bWp.angle = wp.angle;
				bWp.cy = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, bWp.cx,0,bWp.cz);
				bWp.wait = false;
				bWp.rev = false;
				bWp.crossing = false;
				bWp.generated = true;
				if k < numPoints and kk == N then
					if finalCourse[k+1].workstart then
						bWp.turnStart = true
						if courseplay.geometry:ccw(finalCourse[k-1], finalCourse[k], finalCourse[k+1]) then
							bWp.turn = "left";
						else
							bWp.turn = "right";
						end;				
					end;
				end;
 				table.insert(wayPoints,bWp);
			end;
		else --needed?!
			lastWorkStart = false;
		end;
		
	end;
	
	 -- .wait = true
	 -- wayPoints[1].wait = true;
	 -- wayPoints[#wayPoints].wait = true;
	 
	return wayPoints;
end;

function courseplay.geometry:mergeFinalCourse(workLines,startPointMarker)
	courseplay:debug("assembling final course...",7)
	local final = {};
	-- workLines[variant][group][1].cx
	local Ngroups = #workLines[1];
	local function merge(a,b) --merges b into a. destroys original data
		-- local new = {};
		while b~= nil and #b>0 do
			table.insert(a,b[1])
			table.remove(b,1)
		end;
	end;
	
	local available = {}
	for k=1,Ngroups do
		available[k]=true;
	end;
	
	
	-- if false then
		-- --------------- this piece of code is OLD: it takes the largest group and defaults variant to 1:
		-- local currentGroup = 0;
		-- local currentGroupSize = 0;
		-- for group=1,Ngroups do --start with largest group
			-- local Npoints = #workLines[1][group];
			-- if Npoints>currentGroupSize then
				-- currentGroupSize = Npoints;
				-- currentGroup = group;
			-- end;
		-- end;
		
		-- local variant = 1; -- what is a good way to determine a starting variant?!
		
		-- merge(final,workLines[variant][currentGroup])
		-- available[currentGroup] = false;
	
	-- end;
	
	
	
	local lastX = startPointMarker.cx;
	local lastZ = startPointMarker.cz;
	
	-- local coursesLeft = Ngroups-1;
	local coursesLeft = Ngroups;
	while coursesLeft>0 do
		--find nearest neighbor:
		local distance2Found = math.huge
		local groupFound = 0;
		local variantFound = 0;
		-- local lastX = final[#final].cx;
		-- local lastZ = final[#final].cz;
		
		coursesLeft = 0;
		for group,isLeft in pairs(available) do
			if isLeft then
				coursesLeft = coursesLeft+1;
				for variant = 1,4 do
					if group~= nil and variant~= nil and workLines[variant]~= nil and workLines[variant][group]~= nil and #workLines[variant][group]>0 and workLines[variant][group][1].cx~= nil then
						local testX = workLines[variant][group][1].cx;
						local testZ = workLines[variant][group][1].cz;
						local dis2 = (lastX-testX)^2 + (lastZ-testZ)^2;
						if dis2 < distance2Found then
							distance2Found = dis2
							groupFound = group;
							variantFound = variant;
						end;					
					end;
				end;				
			end;
		end;
		
		courseplay:debug("nearest group found: "..tostring(groupFound).." variant: "..tostring(variantFound),7)
		workLines[variantFound][groupFound][1].doLineCheck = true;			
		merge(final,workLines[variantFound][groupFound])
		available[groupFound] = false;
		coursesLeft = coursesLeft-1; --thats the one we just found..
		lastX = final[#final].cx;
		lastZ = final[#final].cz;
		
	end;
	
	courseplay:debug("final course has "..tostring(#final).." points",7)
	
	return final
end;

function courseplay.geometry:fillSubFieldCourses(groupPolys,width, groupAngles, variant)
	variant = Utils.getNoNil(variant,1);
	local sideFactor = 1;
	local upFactor = 1;
	if variant >2 then
		upFactor = -1;
	end;
	if variant%2 == 0 then
		sideFactor = -1;
	end;
	courseplay:debug("filling subField with upFactor "..tostring(upFactor).." and sideFactor "..tostring(sideFactor),7)
	local Ngroups = #groupPolys;
	local workLinesAssembled = {};
	-- local groupPoly
	for group = 1,Ngroups do
				
		local groupPoly = groupPolys[group];
				
		local yRot = groupAngles[group];
		local dx, dz = Utils.getDirectionFromYRotation(yRot);
		-- dx, dz = upFactor*dx, upFactor*dz; 
		local area,centroid, maxRadius = courseplay.geometry:getAreaCentroidMaxcircle(groupPoly)
		local workingLines = {}
		-- local invert = false;
		for sideShift = -sideFactor*maxRadius,sideFactor*maxRadius,sideFactor*width do
			local pointX = centroid.x - sideShift*dz;
			local pointZ = centroid.z + sideShift*dx;
			local res = courseplay.geometry:intersectionsWithRay(groupPoly,pointX,pointZ, dx,dz);
			if #res > 0 then
				for kk = 1,#res,2 do
					local ind1 = kk;
					local ind2 = kk+1;
					-- courseplay.geometry:shrinkIntersection(res,ind1,0.1);
					if upFactor<0 then
						ind1, ind2 = ind2, ind1;
					end;
					local x1 = pointX + dx*res[ind1];
					local z1 = pointZ + dz*res[ind1];
					local x2 = pointX + dx*res[ind2];
					local z2 = pointZ + dz*res[ind2];
					local newPoint = {};
					newPoint[1] = {cx=x1, cz=z1};
					newPoint[2] = {cx=x2, cz=z2};
					newPoint.multiplier = #res/2;
					newPoint.multiplierIndex = (kk+1)/2;
					-- newPoint.invert = invert;
					-- newPoint.isMulti = #res>2;
					-- invert = not invert;					
					table.insert(workingLines,newPoint);
				end;
			end;
		end;
		
	
		local index = 1;
		while #workingLines>0 do
			if workingLines[index].multiplier == 1 then
				index = index+1;
			else
				local Nblock = 1;
				if index < #workingLines then				
					while workingLines[index+Nblock].multiplier > 1 do
						Nblock = Nblock+1;
						if index+Nblock > #workingLines then
							Nblock = Nblock-1; --really? check it!
							break;
						end;
					end;
				end;
				courseplay:debug("block with "..tostring(Nblock).." points found",7)
				
				local direction = 1.0;
				if index > 1 then
					local lastPoint = workingLines[index-1];
					local point = workingLines[index];
					local distance1 = (point[1].cx-lastPoint[2].cx)^2 + (point[1].cz-lastPoint[2].cz)^2;
					local distance2 = (point[2].cx-lastPoint[2].cx)^2 + (point[2].cz-lastPoint[2].cz)^2;
					if distance2 < distance1 then
						direction = -1;
					end;
				end;
				-- print("sorting direction: "..tostring(direction))
				for jj = index,index+Nblock-2 do --it has been soo long, it has to be a bubble sort ;)
					for jjj = jj+1,index+Nblock-1 do
						if direction*workingLines[jj].multiplierIndex > direction*workingLines[jjj].multiplierIndex then --swap
							workingLines[jj], workingLines[jjj] = workingLines[jjj], workingLines[jj];
						end;
					end;
				end;
				
				for jj = index,index+Nblock-1 do
					-- print(tostring(jj).." "..tostring(workingLines[jj].multiplier).." "..tostring(workingLines[jj].multiplierIndex))
					workingLines[jj].multiplier = 1;
					workingLines[jj].doLineCheck = true;
				end;
				
				
				
				index = index + Nblock;
			end;
			
			if index > #workingLines then
				break;
			end;
		end;
		
		
		
		local workLineAssembled = {};
		local lastMPI = 0;
		for index,line in pairs(workingLines) do
			if line.multiplier == 1 then
				local distance1 = 0;
				local distance2	= 1;
				if index > 1 then
					local lastPoint = workLineAssembled[#workLineAssembled];
					distance1 = (line[1].cx-lastPoint.cx)^2 + (line[1].cz-lastPoint.cz)^2;
					distance2 = (line[2].cx-lastPoint.cx)^2 + (line[2].cz-lastPoint.cz)^2;
				end;
				if line.doLineCheck or lastMPI~=0 then
					if line.doLineCheck then
						if lastMPI~=line.multiplierIndex then
							lastMPI = line.multiplierIndex;
						else
							line.doLineCheck = false;
						end;
					else --need to catch 1st point after:
						line.doLineCheck = true;
						lastMPI = 0;
					end;
				-- else
					-- lastMPI = 0; --probably redundant
				end;
				
				if distance1 < distance2 then
					table.insert(workLineAssembled,{cx = line[1].cx, cz = line[1].cz, workstart=true,  doLineCheck=line.doLineCheck, multiplierIndex=line.multiplierIndex})	--sort this HERE (morgen...)			
					table.insert(workLineAssembled,{cx = line[2].cx, cz = line[2].cz, workstart=false, doLineCheck=false, multiplierIndex=line.multiplierIndex})
				else
					table.insert(workLineAssembled,{cx = line[2].cx, cz = line[2].cz, workstart=true,  doLineCheck=line.doLineCheck, multiplierIndex=line.multiplierIndex})
					table.insert(workLineAssembled,{cx = line[1].cx, cz = line[1].cz, workstart=false, doLineCheck=false, multiplierIndex=line.multiplierIndex})				
				end;
			else
				
			end;
			
		end;
		table.insert(workLinesAssembled,workLineAssembled)
		-- print("work line:")
		-- for key,line in pairs(workLineAssembled) do
			-- print("group: "..tostring(group).." line: "..tostring(key).." workstart: "..tostring(line.workstart).." doLineCheck: "..tostring(line.doLineCheck).." multiplierIndex: "..tostring(line.multiplierIndex))
		-- end;
		
		
	end;
	
	
	
	return workLinesAssembled;
end;

function courseplay.geometry:getOptimalCourse(split,speed, width, turningTime, hoppingTime)
	local areas_lookup = {};
	local centroid_lookup = {};
	local maxRadius_lookup = {};
	
	for subField,poly in pairs(split) do
		courseplay:debug(tostring(subField).." "..tostring(#poly),7)
		local area,centroid, maxRadius = courseplay.geometry:getAreaCentroidMaxcircle(poly)
		areas_lookup[subField] = area;
		centroid_lookup[subField] = centroid;
		maxRadius_lookup[subField] = maxRadius;
	end;
	local edge_lookup = {};
	local edge_orientation_lookup = {};
	for i = 1,#split do
		edge_lookup[i] = {};
		edge_orientation_lookup[i] = {};
		for k = i,#split,1 do
			if edge_lookup[k] == nil then
				edge_lookup[k] = {};
				edge_orientation_lookup[k] = {};
			end;
			if k == i then
				edge_lookup[i][i] = 0;
				edge_orientation_lookup[i][i] = 0;
			else
				local distance, yRot = courseplay.geometry:getSharedEdgeLength(split[i],split[k])
				edge_lookup[i][k] = distance;
				edge_lookup[k][i] = distance;
				edge_orientation_lookup[i][k] = yRot;
				edge_orientation_lookup[k][i] = yRot;
			end;
		end;
	end;
	------------------
	
	local rotYvec = {}
	for rot = 0,math.pi-0.0001,math.pi/45 do     --this is where angle quantization is taking place!
	-- for rot = 0,math.pi-0.0001,math.pi/450 do
	-- for rot = 0,math.pi-0.0001,math.pi/4 do
		table.insert(rotYvec,rot)
	end;
	
	for subField,poly in pairs(split) do --add angles of edges and their ortgononals
		local last_k = #poly;
		local k = 1;
		while k <= #poly do
			local dx = poly[k].cx - poly[last_k].cx;
			local dz = poly[k].cz - poly[last_k].cz;
			local yRot = Utils.getYRotationFromDirection(dx, dz);
			table.insert(rotYvec,yRot);
			table.insert(rotYvec,yRot+math.pi/2);
			
			last_k = k;
			k = k+1;
		end;		
	end;
	
	-- print("angles used for optimization:")
	-- for key,value in pairs(rotYvec) do
		-- print(tostring(key).." "..tostring(value*180/math.pi))
	-- end;
	
	
	local cost_lookup = {};
	local optAngle_lookup = {};
	
	for subField,poly in pairs(split) do
		-- local area,centroid, maxRadius = courseplay.geometry:getAreaCentroidMaxcircle(poly)
		local centroid = centroid_lookup[subField];
		local maxRadius = maxRadius_lookup[subField];
		
		cost_lookup[subField] = {}
		optAngle_lookup[subField] = {}
		
		
		
		local optValue = math.huge;
		for k,rotY in pairs(rotYvec) do
			local cost = courseplay.geometry:getSubCourseCost(poly,rotY,centroid,maxRadius,speed, width, turningTime);
			-- print(tostring(subField).." ".." "..tostring(rotY*180/math.pi).." "..tostring(cost))
			cost_lookup[subField][k] = cost;
			-- print(tostring(subField).." "..tostring(rotY*180/math.pi).." "..tostring(cost))
			
			if cost < optValue then
				optValue = cost;
				optAngle_lookup[subField].optAngle = rotY;
				optAngle_lookup[subField].optCost = cost;
			end;
		end;
	end;
	
	----------------------------
	------ this is all lookup stuff so far. optimization starts here:
	
	local groupListByField = {};
	local groupListByGroup = {};
	local currentGroup = 0;
	local groupAngles = {};
	local groupCosts = {};
	
	
	for i = 1,#split do
		if groupListByField[i] == nil or groupListByField[i] == currentGroup then
			if groupListByField[i] == nil then
				currentGroup = currentGroup+1;
				groupListByField[i] = currentGroup;
				groupAngles[currentGroup] = optAngle_lookup[i].optAngle;
				groupCosts[currentGroup] = optAngle_lookup[i].optCost;
				groupListByGroup[currentGroup] = {};
				table.insert(groupListByGroup[currentGroup],i);
				courseplay:debug("subField "..tostring(i).." starts new group "..tostring(currentGroup),7)
				courseplay:debug("angle set to :"..tostring(groupAngles[currentGroup]*180/math.pi),7)
			end;
		
			for k = 1,#split do
				if k~=i then
					if groupListByField[k] == nil then --k is still available
						local L = edge_lookup[i][k];
						if L > 0 then --k is a neighbor of i
							-- local minCostGroup = optAngle_lookup[i].optCost + optAngle_lookup[k].optCost + hoppingTime; --dont use optAngle_lookup[i].optCost, but take lookup from group!!
							local minCostGroup = groupCosts[currentGroup] + optAngle_lookup[k].optCost + hoppingTime; --dont use optAngle_lookup[i].optCost, but take lookup from group!!
							courseplay:debug(tostring(i).." "..tostring(k).." "..tostring(L).." "..tostring(minCostGroup),7)
							
							local yRotEdge = edge_orientation_lookup[i][k];
							local yRotEdge1 = math.sin(yRotEdge);
							local yRotEdge2 = math.cos(yRotEdge);
							

							
							local minValueFound = math.huge;
							local angle12 = 0; --this is the index, not the value
							for a,yrot in pairs(rotYvec) do
								-- local cost = cost_lookup[i][a] + cost_lookup[k][a];
								local cost = cost_lookup[k][a];
								for _,ii in pairs(groupListByGroup[currentGroup]) do
									cost = cost + cost_lookup[ii][a];
								end;
								
								local cosAlpha = yRotEdge1*math.sin(yrot) + yRotEdge2*math.cos(yrot);
								local sinAlpha = math.sin(math.acos(cosAlpha));
								cost = cost - 2*math.abs(sinAlpha)*L/width*turningTime;
								
								if cost < minValueFound then
									minValueFound = cost;
									angle12 = a;
								end;								
							end;

							-- minValueFound = minValueFound - 2*L/width*turningTime;  this depends on angle!!! STILL MISSING
							if minValueFound < minCostGroup then
								groupListByField[k] = currentGroup;
								groupAngles[currentGroup] = rotYvec[angle12];								
								groupCosts[currentGroup] = minValueFound;
								table.insert(groupListByGroup[currentGroup],k);
																
								courseplay:debug("putting subField "..tostring(k).." into group "..tostring(currentGroup),7)
								courseplay:debug("new value: "..tostring(minValueFound).." was found superior to "..tostring(minCostGroup),7)
								courseplay:debug("angle set to :"..tostring(groupAngles[currentGroup]*180/math.pi),7)
								
							end;							
						end;
					end;
				end;
			end;
		end;
	end;
	
	courseplay:debug("merging fields into groups",7)
	local groupPolys = {}
	for group = 1, #groupListByGroup do
		local groupPoly = split[groupListByGroup[group][1]];
		for k = 2,#groupListByGroup[group] do
			groupPoly = courseplay.geometry:mergedWith(groupPoly,split[groupListByGroup[group][k]])
		end;
		groupPolys[group] = groupPoly;
	end;
	
	
	local maxGroups = 20; --this is an IMPORTANT control parameter...
	
	
	while true do   --merge groups
		local blocked = {}
		courseplay:debug("thinking about merging groups...",7)
		local reDo = true;
		while reDo do
			reDo = false;
			-- local insertPoint = 0;
			local mergeGroups = {};
			for k=1,#groupPolys-1 do
				for kk = k+1,#groupPolys do
					-- find common edge
					-- print(tostring(k).." "..tostring(kk).." "..tostring(#groupPolys))
					
					local edgeFound = false;
					local L, edgeAngle = courseplay.geometry:getSharedEdgeLength(groupPolys[k],groupPolys[kk]);
					if Utils.getNoNil(L,0) > 0 then
						edgeFound = true;					
						-- break;
					end;
					-- for _,subFieldGroup1 in pairs(groupListByGroup[k]) do				
						-- for _,subFieldGroup2 in pairs(groupListByGroup[kk]) do
							-- print(subFieldGroup1," ",subFieldGroup2)
							-- -- print(tostring(edge_lookup[subFieldGroup1][subFieldGroup2]))
							-- local L= courseplay.geometry:getSharedEdgeLength(split[subFieldGroup1],split[subFieldGroup2]);
							-- print(tostring(L));
							-- if Utils.getNoNil(L,0) > 0 then
								-- edgeFound = true;
								-- -- insertPoint = searchIndex;
								-- break;
							-- end;
						-- end;
									
					if edgeFound then
						courseplay:debug("groups "..tostring(k).." and "..tostring(kk).." have a common edge",7)
						
						local recalcGroup = kk;
						local angle = groupAngles[k];
						local oldAngle = groupAngles[kk];
						if groupCosts[k] < groupCosts[kk] then
							recalcGroup = k;
							angle = groupAngles[kk];
							oldAngle = groupAngles[k];
						end;
						
						--find index of angle						
						local a = 0;
						for k,rotY in pairs(rotYvec) do
							if rotY == angle then
								a = k;
								break;
							end;
						end;
						
						if groupAngles[k] ==  groupAngles[kk] then
							courseplay:debug("same angle! -> merging",7)
							mergeGroups[k]=kk;
							reDo = true;
							break;
						else
							local oldCost = groupCosts[recalcGroup];
							local newCost = 0;
							
							for _,subField in pairs(groupListByGroup[recalcGroup]) do
								-- angle
								newCost = newCost + cost_lookup[subField][a];
							end;
							
							local edgeComp = -L*((math.cos(edgeAngle-angle))^2-(math.cos(edgeAngle-oldAngle))^2)/width*turningTime
							courseplay:debug("edge compensation: "..tostring(edgeComp),7)
							courseplay:debug("hoppingTime: "..tostring(hoppingTime),7)
							newCost = newCost - edgeComp - hoppingTime;
							
							courseplay:debug("oldcost: "..tostring(oldCost).."  newCost: "..tostring(newCost),7)
							-- if true and oldCost - newCost < hoppingTime then
							if newCost - oldCost < 0 then
								courseplay:debug("merging! due to cost",7)
								mergeGroups[k]=kk;
								reDo = true;
								
								break;
							else
								courseplay:debug("no deal!",7)
							end;
						end;
					end;
					
				end;
				if reDo then
					break;
				end;
			end;
			
			if reDo then
				for group1, group2 in pairs(mergeGroups) do
					
					-- for _,field in pairs(groupListByGroup[group2]) do
						-- table.insert(groupListByGroup[group1],insertPoint+1,field);
					-- end;
					courseplay:debug("merging group "..tostring(group1).." and group "..tostring(group2),7)
					local newMerge = courseplay.geometry:mergedWith(groupPolys[group1],groupPolys[group2])
					table.remove(groupPolys,group2)
					groupPolys[group1] = newMerge;
					
					table.remove(groupAngles,group2)				
				end;
			end;
		end;
		if #groupPolys <= maxGroups then
			break;
		else
			courseplay:debug("#groupPolys > maxGroups ("..tostring(#groupPolys)..">"..tostring(maxGroups),7)
			hoppingTime = hoppingTime*1.1;
			courseplay:debug("setting hoppingTime to "..tostring(hoppingTime),7)
		end;
	end;	
	
	
	courseplay:debug("done",7)
	return groupPolys, groupAngles								
					
end;

function courseplay.geometry:addInFieldPoints(course,outline)
	courseplay:debug("checking course with "..tostring(#course).." points",7);
	for k = 1,#course-1 do
		local point1 = course[k]
		local point2 = course[k+1]
		-- if point1.workstart then
			-- -- print("workpoint")
			-- -- courseplay.geometry:isLineLeavingField(point1,point2,outline)
		-- else
		if point2.doLineCheck then
			courseplay:debug("checking point "..tostring(k),7)
			local distance2 = (point1.cx+point2.cx)^2 + (point1.cz+point2.cz)^2;
			-- print(distance2)
			-- print(courseplay.geometry:isLineLeavingField(point1,point2,outline))
			
			if distance2 > 200 and courseplay.geometry:isLineLeavingField(point1,point2,outline) then --value 200 is placeholder for 2.5 working width (squared). do not crop if shortcut is small
				courseplay:debug("correcting corner cutting or indention at point "..tostring(k),7)
				
				
				local k1 = courseplay.geometry:findNearestPointIndex(point1,outline)
				local k2 = courseplay.geometry:findNearestPointIndex(point2,outline)
				courseplay:debug("outline start point2: "..tostring(k1).." "..tostring(k2),7)
				if k1~= nil and k2~= nil then
					if k1 == k2 then --single point, just add
						courseplay:debug("single point correction is sufficient, inserting outline point "..tostring(k1),7)					
						table.insert(course,k+1,outline[k1]);
					else
						courseplay:debug("searching between "..tostring(k1).." and "..tostring(k2),7)
						local step = 1;
						-- if k2 < k1 then 
							-- step = -1; 
						-- end;
						local dirVec1 = {};
						local dirVec2 = {};
						local indexSearch = k1;
						
						while indexSearch ~= k2 do						
							table.insert(dirVec1,indexSearch);
							indexSearch = indexSearch + step;
							if indexSearch > #outline then
								indexSearch = 1;
							elseif indexSearch < 1 then
								indexSearch = #outline;
							end;
						end;
						table.insert(dirVec1,k2);
						step = -1;
						indexSearch = k1;					
						while indexSearch ~= k2 do						
							table.insert(dirVec2,indexSearch);
							indexSearch = indexSearch + step;
							if indexSearch > #outline then
								indexSearch = 1;
							elseif indexSearch < 1 then
								indexSearch = #outline;
							end;
						end;
						table.insert(dirVec2,k2);
						courseplay:debug("dirVecs: "..tostring(#dirVec1).."   "..tostring(#dirVec2),7)
						if #dirVec1 > #dirVec2 then
							dirVec1 = dirVec2;
						end;
						
						if true then --lets see if we can not shorten even further:
							courseplay:debug("trying to shorten route extension:",7)
							while true do
								if #dirVec1>1 then
									if not courseplay.geometry:isLineLeavingField(point1,outline[dirVec1[2]],outline) then
										courseplay:debug("removing 1st point",7)
										table.remove(dirVec1,1);										
									else
										break;
									end;
								else --down to 1 point and we already have decided we need the round tour. finished
									break;
								end;
							end;

							while true do
								if #dirVec1>1 then
									if not courseplay.geometry:isLineLeavingField(point2,outline[dirVec1[#dirVec1-1]],outline) then
										courseplay:debug("removing last point",7)
										table.remove(dirVec1,#dirVec1);										
									else
										break;
									end;
								else --down to 1 point and we already have decided we need the round tour. finished
									break;
								end;
							end;							
						end;
						
						for kk = #dirVec1,1,-1 do
						-- for kk = 1,#dirVec1 do
							courseplay:debug(kk," ",dirVec1[kk],7)
							table.insert(course,k+1,outline[dirVec1[kk]]);
						end;
					end;
				else
					courseplay:debug("addInFieldPoints: could not find outline points! This is mathematically impossible, so there must be some problem",7)
				end;
				courseplay:debug("finished corner cutting or indention at point "..tostring(k),7)
			end;
		end;
		
	end;
	return course;
end;

function courseplay.geometry:findNearestPointIndex(point,outline)
	local x = point.cx;
	local z = point.cz;
	
	local minOk = math.huge;
	local kOpt
	for k = 1, #outline do
		if not courseplay.geometry:isLineLeavingField(point,outline[k],outline) then
			local distance2 = (x-outline[k].cx)^2 + (z-outline[k].cz)^2
			-- minOk = math.min(minOk,distance2);
			if distance2 < minOk then
				kOpt = k;
				minOk = distance2;
			end;
		else
			-- print("index "..tostring(k).." is not directly reachable from start point")
		end;
	end;
	return kOpt;
end;

function courseplay.geometry:isLineLeavingField(point1,point2,outline)
	local dx = point2.cx-point1.cx;
	local dz = point2.cz-point1.cz;	
	
	local L = math.sqrt(dx^2+dz^2);
	dx = dx/L;
	dz = dz/L;
	local res,cosAngle = courseplay.geometry:intersectionsWithRay(outline,point1.cx,point1.cz, dx,dz,true);
		
	local score = 0;
	for k=0,1,0.1 do
		local xTest = point1.cx + k*L*dx;
		local zTest = point1.cz + k*L*dz;		
		if not courseplay.geometry:contains(outline,xTest,zTest) then
			-- print("test")
			local radius = 0.5;
			if not courseplay.geometry:contains(outline,xTest+radius*dz,zTest-radius*dx) and not courseplay.geometry:contains(outline,xTest-radius*dz,zTest+radius*dx) then --back to 'topfschlagen'...
				-- print("out of field found at k="..tostring(k))
				score = score + 1;
				-- return true;
			end;
		end;
	end;
	
	-- if score > 1 then
		-- print("isLineLeavingField score: "..tostring(score))
	-- end;
	if score > 6 then
		-- print("score: "..tostring(score))
		return true;
	end;
		
	for key,value in pairs(res) do
		if math.abs(cosAngle[key]) < 0.98 then
			if value > 2 and value  < L-2 then
				-- print("isLineLeavingField intersectionsWithRay value: "..tostring(value).."/"..tostring(L))
				return true
			end
		end;
	end;
	
	return false;
	
	
	
	
	
	-- so is the very naive (and pissed off) version:
	-- -- for k=0.02,.98,0.1 do
		-- -- local xTest = point1.cx + k*L*dx;
		-- -- local zTest = point1.cz + k*L*dz;		
		-- -- if not courseplay.geometry:contains(outline,xTest,zTest) then
			-- -- print("test")
			-- -- if not courseplay.geometry:contains(outline,xTest+2*dz,zTest-2*dx) and not courseplay.geometry:contains(outline,xTest-2*dz,zTest+2*dx) then --back to 'topfschlagen'...
				-- -- print("out of field found at k="..tostring(k))
				-- -- return true;
			-- -- end;
		-- -- end;
	-- -- end;
	-- -- return false;
	-- this is neither elegant nor is it guarantied to be true, but it should work better than all the other stuff below
	
	
	-- local res,cosAngle = courseplay.geometry:intersectionsWithRay(outline,point1.cx,point1.cz, dx,dz,true);
	-- print("a",#res)
	-- for key,value in pairs(res) do
		-- print(tostring(key).." "..tostring(value).." "..tostring(cosAngle[key]))
	-- end;
	
	-- for k=#res,1,-1 do --kick out shit
		-- local value = res[k];
		-- -- if value < -0.01 or value > 1.01 then 
		-- if value < 0.01 or value > 0.99 then 
			-- table.remove(res,k)
			-- table.remove(cosAngle,k)
		-- elseif math.abs(cosAngle[k]) > 0.98 then
			-- table.remove(res,k)
			-- table.remove(cosAngle,k)
		-- end;
	-- end;
	-- print("b",#res)
	-- if #res == 0 then
		-- return false;
	-- elseif #res%2 ~= 0 or #res > 2 then
		-- print("leaving point found, #res: "..tostring(#res))
		-- for key,value in pairs(res) do
			-- print(tostring(key).." "..tostring(value))
		-- end;
		-- return true;
	-- end;
	
	-- local isLeaving = false;
	-- -- for k=1,#res,2 do --,value in pairs(res) do
		-- -- local value = res[k];
		
	-- local lambda = (res[1]+res[2])/2;
	-- local xTest = point1.cx + lambda*dx;
	-- local zTest = point1.cz + lambda*dz;
	
	-- isLeaving = not courseplay.geometry:contains(outline,xTest,zTest)
	
	-- if isLeaving then
		-- print("leaving point found: killed by courseplay.geometry:contains()")
	-- end;
		
	-- -- end;	
	-- return isLeaving;
end;

function courseplay.geometry:contains(v,x,y)	
	-- test if an edge cuts the ray
	local function cut_ray(p,q)
		return ((p.cz > y and q.cz < y) or (p.cz < y and q.cz > y)) -- possible cut
				and (x - p.cx < (y - p.cz) * (q.cx - p.cx) / (q.cz - p.cz)) -- x < cut.cx
	end
	-- test if the ray crosses boundary from interior to exterior.
	-- this is needed due to edge cases, when the ray passes through
	-- polygon corners
	local   function cross_boundary(p,q)
				return (p.cz == y and p.cx > x and q.cz < y)
						or (q.cz == y and q.cx > x and p.cz < y)
			end

	local in_polygon = false
	local p,q = v[#v],v[#v]
	for i = 1, #v do
		p,q = q,v[i]
		if cut_ray(p,q) or cross_boundary(p,q) then
			in_polygon = not in_polygon
		end
	end
	return in_polygon
end

function courseplay.geometry:getSubCourseCost(poly,yRot,centroid,maxRadius,speed, width, turningTime)
	local cost = 0;
	local dx, dz = Utils.getDirectionFromYRotation(yRot);
	
	for sideShift = -maxRadius,maxRadius,width do
		local pointX = centroid.x - sideShift*dz;
		local pointZ = centroid.z + sideShift*dx;
		local res, cosAngle = courseplay.geometry:intersectionsWithRay(poly,pointX,pointZ, dx,dz,true);
		if #res > 0 then
			if #res~=2 then --we could handle this, but we dont want to at the moment
				courseplay:debug("subfield for getSubCourseCost is NOT CONVEX!",7)
			end;
			-- print("courseCost: cosA1 "..tostring(cosAngle[1]).." ".." cosA2 "..tostring(cosAngle[2]))
			local distance = math.abs(res[1]-res[2]);
			local incidenceFac = 1 + 7.5*(cosAngle[1]^2+cosAngle[2]^2); --factor 7.5 here is arbitrary and can (should?) be adjusted
			cost = cost + distance/speed + incidenceFac*turningTime;
		
		end;
	end;
	
	return cost;
end;

-- here starts the real algorithm library:

-- test wether a and b lie on the same side of the line c->d
function courseplay.geometry:onSameSide(a,b, c,d)
	local px, py = d.cx-c.cx, d.cz-c.cz
	local l = courseplay.geometry:det(px,py, a.cx-c.cx, a.cz-c.cz)
	local m = courseplay.geometry:det(px,py, b.cx-c.cx, b.cz-c.cz)
	return l*m >= 0
end

function courseplay.geometry:pointInTriangle(p, a,b,c)
	return courseplay.geometry:onSameSide(p,a, b,c) and courseplay.geometry:onSameSide(p,b, a,c) and courseplay.geometry:onSameSide(p,c, a,b)
end

function courseplay.geometry:segmentsInterset(a,b, p,q)
	return not (courseplay.geometry:onSameSide(a,b, p,q) or courseplay.geometry:onSameSide(p,q, a,b))
end;

-- test whether any point in vertices (but pqr) lies in the triangle pqr
-- note: vertices is *set*, not a list!
function courseplay.geometry:anyPointInTriangle(vertices, p,q,r)
	for v in pairs(vertices) do
		if v ~= p and v ~= q and v ~= r and courseplay.geometry:pointInTriangle(v, p,q,r) then
			return true
		end
	end
	return false
end
-- test is the triangle pqr is an "ear" of the polygon
-- note: vertices is *set*, not a list!
function courseplay.geometry:isEar(p,q,r, vertices)
	return courseplay.geometry:ccw(p,q,r) and not courseplay.geometry:anyPointInTriangle(vertices, p,q,r)
end



function courseplay.geometry:getSharedEdgeLength(poly,other)
	local p,q = courseplay.geometry:getSharedEdge(poly,other)
	if p == nil or q == nil then --no shared edge
		return 0;
	end;
	
	local length = 0;
	
	local point1 = poly[p];
	local index2 = p+1;
	if p == #poly then
		index2 = 1;
	end;
	local point2 = poly[index2];
	
	local distance = (point1.cx-point2.cx)^2 + (point1.cz-point2.cz)^2;
	distance = math.sqrt(distance);
	
	local yRot = Utils.getYRotationFromDirection(point1.cx-point2.cx, point1.cz-point2.cz);
	return distance, yRot;
end;


-- returns starting/ending indices of shared edge, i.e. if p and q share the
-- edge with indices p1,p2 of p and q1,q2 of q, the return value is p1,q2
function courseplay.geometry:getSharedEdge(p,q)
	local pindex = setmetatable({}, {__index = function(t,k)
		local s = {}
		t[k] = s
		return s
	end})
	-- record indices of vertices in p by their coordinates
	for i = 1,#p do
		pindex[p[i].cx][p[i].cz] = i
	end
	-- iterate over all edges in q. if both endpoints of that
	-- edge are in p as well, return the indices of the starting
	-- vertex
	local i,k = #q,1
	for k = 1,#q do
		local v,w = q[i], q[k]
		if pindex[v.cx][v.cz] and pindex[w.cx][w.cz] then
			return pindex[w.cx][w.cz], k
		end
		i = k
	end	
end


-- return merged polygon if possible or nil otherwise
function courseplay.geometry:mergedWith(poly,other)
	local p,q = courseplay.geometry:getSharedEdge(poly,other)
	assert(p and q, "Polygons do not share an edge")
	local ret = {}
	for i = 1,p-1 do
		ret[#ret+1] = poly[i].cx
		ret[#ret+1] = poly[i].cz
		end
	for i = 0,#other-2 do
		i = ((i-1 + q) % #other) + 1
		ret[#ret+1] = other[i].cx
		ret[#ret+1] = other[i].cz
	end
	for i = p+1,#poly do
		ret[#ret+1] = poly[i].cx
		ret[#ret+1] = poly[i].cz
	end
	return courseplay.geometry:newPolygon(unpack(ret))
end

-- split polygon into convex polygons.
-- note that this won't be the optimal split in most cases, as
-- finding the optimal split is a really hard problem.
-- the method is to first triangulate and then greedily merge
-- the triangles.
function courseplay.geometry:splitConvex(poly)
	-- edge case: polygon is a triangle or already convex
	if #poly <= 3 or courseplay.geometry:isConvex(poly) then return {poly} end
	
	local convex = courseplay.geometry:triangulate(poly)
	local i = 1
	repeat
		local p = convex[i]
		local k = i + 1
		while k <= #convex do
			local success, merged = pcall(function() return courseplay.geometry:mergedWith(p,convex[k]) end)
			-- if not success then --pcall masks other errors!
				-- print(merged)
			-- end;

			if success and courseplay.geometry:isConvex(merged) then
				convex[i] = merged
				p = convex[i]
				table.remove(convex, k)
			else
				k = k + 1
			end
		end
		i = i + 1
	until i >= #convex
	return convex
end

-- triangulation by the method of kong
function courseplay.geometry:triangulate(poly)
	if #poly == 3 then return {poly} end
	
	local vertices = poly
	local next_idx, prev_idx = {}, {}
	
	
	for i = 1,#vertices do
		next_idx[i], prev_idx[i] = i+1,i-1
	end
	next_idx[#next_idx], prev_idx[1] = 1, #prev_idx
	
	local concave = {}
	for i, v in ipairs(vertices) do
		if not courseplay.geometry:ccw(vertices[prev_idx[i]], v, vertices[next_idx[i]]) then
			concave[v] = true
		end
	end
	local triangles = {}
	local n_vert, current, skipped, next, prev = #vertices, 1, 0
	while n_vert > 3 do
		next, prev = next_idx[current], prev_idx[current]
		local p,q,r = vertices[prev], vertices[current], vertices[next]
		if courseplay.geometry:isEar(p,q,r, concave) then
			triangles[#triangles+1] = courseplay.geometry:newPolygon(p.cx,p.cz, q.cx,q.cz, r.cx,r.cz)
			next_idx[prev], prev_idx[next] = next, prev
			concave[q] = nil
			n_vert, skipped = n_vert - 1, 0
		else
			skipped = skipped + 1
			
			assert(skipped <= n_vert, "Cannot triangulate polygon")
		end
		current = next
	end
	next, prev = next_idx[current], prev_idx[current]
	local p,q,r = vertices[prev], vertices[current], vertices[next]
	triangles[#triangles+1] = courseplay.geometry:newPolygon(p.cx,p.cz, q.cx,q.cz, r.cx,r.cz)
	
	return triangles
end

function courseplay.geometry:assertOrientation(poly)
	if courseplay.geometry:getArea(poly) < 0 then
		-- print("reversing order of polygon")
		local newPoly = {};
		for k = #poly,1,-1 do
			table.insert(newPoly,poly[k]);
		end;
		return newPoly;
	else
		return poly;
	end;
end;

function courseplay.geometry:det(x1,y1,x2,y2)
	return x1*y2 - y1*x2;
end;

function courseplay.geometry:getArea(vertices)
	local p,q = vertices[#vertices], vertices[1]
	local det = courseplay.geometry:det(p.cx,p.cz, q.cx,q.cz) -- also used below
	local area = det
	for i = 2,#vertices do
		p,q = q,vertices[i]
		area = area + courseplay.geometry:det(p.cx,p.cz, q.cx,q.cz)
	end
	area = area / 2
	return area
end;

function courseplay.geometry:isConvex(v)
	if courseplay.geometry:isConvexDir(v,1.0) then		
		return true
	elseif courseplay.geometry:isConvexDir(v,-1.0) then
		courseplay:debug("isConvexDir minus true!! polygon is probably turning in wrong direction",7)
		return true
	end;
	return false;
end;

function courseplay.geometry:isConvexDir(v,dir)
	if #v == 3 then return true end
	
	if not courseplay.geometry:ccw(v[#v], v[1], v[2],dir) then
		return false
	end
	for i = 2,#v-1 do
		if not courseplay.geometry:ccw(v[i-1], v[i], v[i+1],dir) then
			return false
		end
	end
	if not courseplay.geometry:ccw(v[#v-1], v[#v], v[1],dir) then
		return false
	end
	return true
end

-- returns true if three points make a counter clockwise turn
function courseplay.geometry:ccw(p, q, r, dir)
	dir = Utils.getNoNil(dir,1.0);
	local x1 = q.cx-p.cx;
	local y1 = q.cz-p.cz;
	local x2 = r.cx-p.cx;
	local y2 = r.cz-p.cz;
	return dir*(x1*y2 - y1*x2) >= 0;
	-- det(x1,y1, x2,y2)
	-- return vector.det(q.x-p.x, q.y-p.y, r.x-p.x, r.y-p.y) >= 0
end;

function courseplay.geometry:intersectionsWithRay(poly,x,y, dx,dy,returnCosAngle)
	local nx,ny = dy, -dx; --vector.perpendicular(dx,dy)
	local wx,xy,det
	local ts = {} -- ray parameters of each intersection
	local q1,q2 = nil, poly[#poly]
	local cosAngle = {};
	
	for i = 1, #poly do
		q1,q2 = q2,poly[i]
		wx,wy = q2.cx - q1.cx, q2.cz - q1.cz
		det = courseplay.geometry:det(dx,dy, wx,wy)
		
		if det ~= 0 then
			-- there is an intersection point. check if it lies on both
			-- the ray and the segment.
			local rx,ry = q2.cx - x, q2.cz - y
			local l = courseplay.geometry:det(rx,ry, wx,wy) / det
			local m = courseplay.geometry:det(dx,dy, rx,ry) / det
			if m >= 0 and m <= 1 then
				-- we cannot jump out early here (i.e. when l > tmin) because
				-- the polygon might be concave
				ts[#ts+1] = l;
				if returnCosAngle then
					local ld = math.sqrt(dx^2+dy^2);
					local lw = math.sqrt(wx^2+wy^2);
					local ca = (wx*dx+wy*dy)/(lw*ld)
					table.insert(cosAngle,ca);
				end;
			end
		else --no this does not catch the parallel routes to border			
			-- lines parralel or incident. get distance of line to
			-- anchor point. if they are incident, check if an endpoint
			-- lies on the ray
			local dist =  (q1.cx-x)*nx + (q1.cz-y)*ny               --courseplay.geometry:dot(q1.cx-x,q1.cz-y, nx,ny)
			if dist == 0 then
				local l =  dx*(q1.cx-x) + dy*(q1.cz-y)         --courseplay.geometry:dot(dx,dy, q1.cx-x,q1.cz-y)
				local m =  dx*(q2.cx-x) + dy*(q2.cz-y)         --courseplay.geometry:dot(dx,dy, q2.cx-x,q2.cz-y)
				if l >= m then
					ts[#ts+1] = l
				else
					ts[#ts+1] = m
				end
				if returnCosAngle then
					local ld = math.sqrt(dx^2+dy^2);
					local lw = math.sqrt(wx^2+wy^2);
					local ca = (wx*dx+wy*dy)/(lw*ld)
					table.insert(cosAngle,ca);
				end;
			end
		end
	end
	if returnCosAngle then
		return ts, cosAngle
	else
		return ts
	end;
end

-- compute polygon area and centroid
function courseplay.geometry:getAreaCentroidMaxcircle(vertices)
	local area, centroid, maxRadius;
	
	
	local p,q = vertices[#vertices], vertices[1]
	local det = courseplay.geometry:det(p.cx,p.cz, q.cx,q.cz) -- also used below
	area = det
	for i = 2,#vertices do
		p,q = q,vertices[i]
		area = area + courseplay.geometry:det(p.cx,p.cz, q.cx,q.cz)
	end
	area = area / 2
	p,q = vertices[#vertices], vertices[1]
	local centroid = {x = (p.cx+q.cx)*det, z = (p.cz+q.cz)*det}
	for i = 2,#vertices do
		p,q = q,vertices[i]
		det = courseplay.geometry:det(p.cx,p.cz, q.cx,q.cz)
		centroid.x = centroid.x + (p.cx+q.cx) * det
		centroid.z = centroid.z + (p.cz+q.cz) * det
	end
	centroid.x = centroid.x / (6 * area)
	centroid.z= centroid.z / (6 * area)
	-- get outcircle
	maxRadius = 0
	for i = 1,#vertices do
		maxRadius = math.max(maxRadius,	math.sqrt((vertices[i].cx - centroid.x)^2 + (vertices[i].cz - centroid.z)^2));		
	end
	
	return area, centroid, maxRadius;	
end;

-- copyright statement from HardOnCollider-project:
--[[
Copyright (c) 2011 Matthias Richter
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
Except as contained in this notice, the name(s) of the above copyright holders
shall not be used in advertising or otherwise to promote the sale, use or
other dealings in this Software without prior written authorization.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
]]

