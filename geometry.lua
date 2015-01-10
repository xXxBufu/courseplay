-- parts of the HardOnCollider-project by Matthias Richter on gitHub are used. Original copyright of this project can be found at the end of this file.

local abs, sqrt = math.abs, math.sqrt;

courseplay.geometry = {};

function courseplay.geometry:angleDeg(p1, p2)
	return math.deg(math.atan2(p2.cz-p1.cz, p2.cx-p1.cx));
end;

function courseplay.geometry:appendArc(points, center, radius, startPoint, endPoint) -- TODO: not used anywhere
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
	local arcSegmentCount = angle * radius / 5;
	local arcAngle = -angle / arcSegmentCount;
	courseplay:debug(string.format('angle is %2.f, %.2f points in arc',math.deg(angle),arcSegmentCount),7);
	table.insert(points, startPoint);
	for i = 1, arcSegmentCount do
		local angle = startAngle + arcAngle * i;
		local point = {
			cx = center.cx + math.cos(angle) * radius,
			cz = center.cz + math.sin(angle) * radius
		};
		table.insert(points, point);
	end;
	table.insert(points, endPoint);
	return points;
end;

function courseplay.geometry:appendPoly(poly1 ,poly2 ,withStart, withEnd) -- appends poly1 to poly2 with without start and end points of poly2
	local startidx = withStart and 1 or 2;
	local endidx = withend and #poly2 or #poly2 - 1;
	for idx = startidx, endidx do --add all but first and last point
		table.insert(poly1, poly2[idx]);
	end;
	return poly1;
end;

function courseplay.geometry:areOpposite(angle1,angle2,tolerance,isDeg) -- TODO: rename "getAnglesAreOpposite"
	if tolerance == nil then tolerance = 0 end;
	if isDeg then
		angle1 = angle1 < 0 and angle1+360 or angle1;
		angle2 = angle2 < 0 and angle2+360 or angle2;
	else
		angle1 = angle1 < 0 and angle1+(math.pi*2) or angle1;
		angle2 = angle2 < 0 and angle2+(math.pi*2) or angle2;
	end;
	local diff = angle1 < angle2 and angle2 - angle1 or angle1 - angle2;
	diff = isDeg and 180 - diff or math.pi - diff;
	courseplay:debug(string.format('[areOpposite] angle1 = %.2f, angle2 = %.2f -> %s',angle1,angle2,tostring(diff<=tolerance)),7);
	return diff <= tolerance;
end;

function courseplay.geometry:arePerpendicular(angle1,angle2,tolerance,isDeg) -- TODO: rename "getAnglesArePerpendicular"
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

function courseplay.geometry:cleanPline(pline,boundingPline,offset,vehicle)
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
	newPline = self:refinePoly(newPline, maxPointDistance);
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

		self:douglasPeucker(points, firstPointNum, indexFurthest, tolerance, pointIndices);
		self:douglasPeucker(points, indexFurthest, lastPointNum, tolerance, pointIndices);
	end;
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
		area = math.abs(area) / 2;
		isClockwise = area < 0;
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
	local pos1, pos2, onEnd = 'NO', 'NO', false;
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
		courseplay:debug(string.format('lineCrossing -> %.2f, %.2f - %s - %s',x,z,pos1,pos2),7);
	end;
	return {
		cx = x,
		cz = z,
		ip1 = pos1,
		ip2 = pos2,
		notOnEnds = not(onEnd)
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
	for i=1, numPoints do
		local cp = pline[i];
		local np = pline[i+1];
		pointInPline = pointInPline * courseplay.utils:crossProductQuery(point, cp, np, true); -- TODO: move to geometry
		local dist, _ = self:pointDistToLine(point,cp,np);
		if dist < minDist then
			minDist = dist;
		end;
	end;
	pointInPline = pointInPline ~= -1 ;
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

function courseplay.geometry:near(v1, v2, tolerance,angle) -- TODO: rename "getAreAnglesClose"
	--returns true if difference between v1 and v2 is under the tolerance value
	local areNear = false;
	if tolerance == nil then tolerance = 0 end;
	if angle then
		local angle1, angle2 = v1, v2;
		if angle == 'deg' then
			tolerance = math.rad(tolerance);
			angle1 = math.rad(angle1);
			angle2 = math.rad(angle2);
		end;
		if math.abs(math.sin(angle1)-math.sin(angle2)) <= math.sin(tolerance) and math.abs(math.cos(angle1)-math.cos(angle2)) <= math.cos(tolerance) then
			--courseplay:debug(string.format('angle1 = %.4f and angle2 = %.4f are near', v1 ,v2 ), 7);
			areNear = true;
		end;
	elseif math.abs(v1 - v2) <= tolerance then
		--courseplay:debug(string.format('v1 = %.4f and v2 = %.4f are near', v1 ,v2 ), 7);
		areNear = true;
	else
		--courseplay:debug(string.format('v1 = %.4f and v2 = %.4f are not near', v1 ,v2 ), 7);
	end;
	return areNear;
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

function courseplay.geometry:offsetPoly(pline, offset, vehicle)
	local pline1 = self:untrimmedOffsetPline(pline, offset);
	pline1 = self:cleanPline(pline1, pline, offset, vehicle);
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

function courseplay.geometry:refinePoly(poly, maxDistance)
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
			newPline = self:appendPoly(newPline, spline, true, false);
		else
			table.insert(newPline,p2);
		end;
	end;
	return newPline;
end;

function courseplay.geometry:samePoints(p1,p2) -- TODO: not used anywhere
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
	local _, _, _, isClockwise = self:getPolygonData(poly, nil, nil, true, true, true); -- TODO: move that fn to geometry
	if isClockwise then
		return poly;
	else
		-- print('reversing order of polygon');
		local newPoly = table.reverse(poly);
		return newPoly;
	end;
end;

function courseplay.geometry:setPolyCounterClockwise(poly)
	local _, _, _, isClockwise = self:getPolygonData(poly, nil, nil, true, true, true); -- TODO: move that fn to geometry
	if isClockwise then
		-- print('reversing order of polygon');
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

	self:douglasPeucker(points, 1, #points, epsilon, remainingPoints);
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
function courseplay.geometry:smoothSpline(refPoint1, startPoint, endPoint , refPoint2 , steps, useC, addHeight)
	if useC == nil then useC = true; end;
	if addHeight == nil then addHeight = false; end;

	local steps = steps or 5;
	local spline = {};
	local x,y,z = 'x','y','z';
	if useC then
		x,y,z = 'cx','cy','cz';
	end;
	local p1,p2,p3,p4;
	if refPoint1 or RefPoint2 then
		p1 = startPoint;
		p4 = endPoint;
		if refPoint1 and refPoint2 then
			local crossingPoint = self:lineIntersection(refPoint1, startPoint, endPoint, refPoint2);
			if crossingPoint.ip1 == 'PFIP' and crossingPoint.ip2 == 'NFIP' then -- crossing point is used as reference
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
				[z] = math.pow(1-t, 3) * p1[z] + 3 * math.pow(1-t, 2) * t * p2[z] + 3 * (1-t) * t*t *  p3[z] + t*t*t * p4[z],
				[y] = addHeight and getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, point[x], 1, point[z]) + 3 or nil
			};
			table.insert(spline, point);
			courseplay:debug(string.format('smoothSpline adding point : %.2f, %.2f', point[x], point[z]), 7);
		end;
	else
		table.insert(spline,startPoint);
		table.insert(spline,endPoint);
	end;
	return spline;
end;

function courseplay.geometry:untrimmedOffsetPline(pline, offset)
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
			table.insert(offPline, crossing);
		else
			table.insert(offPline, s1p2);
			table.insert(offPline, s2p1);
		end;
	end;
	courseplay:debug(string.format('Untrimmed offset finished with %d points', #offPline),7);
	table.remove(pline);
	return offPline;
end;



-- ####################################################################################################
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

