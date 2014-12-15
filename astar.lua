local abs, max, min, pow, sin , huge = math.abs, math.max, math.min, math.pow, math.sin, math.huge;

function courseplay:calcMoves(vehicle,fruitType) 
	local combine = vehicle.cp.activeCombine
	local sx,sy,sz =  getWorldTranslation(vehicle.cp.DirectionNode);
	local tableCw = {}
	local tableCcw = {}
	local outputTable ={}
	local ldX, ldY,ldZ = courseplay:getDriveDirection(combine.cp.DirectionNode, sx,sy,sz)
	local timeoutDistance = courseplay:distanceToPoint(combine, sx,sy,sz) * 2
	local distance = 0
	local angleGrad = math.deg(Utils.getYRotationFromDirection(ldX, ldZ))
	--print("angleGrad = "..tostring(angleGrad))

	local tableCcw = courseplay:scanFruit(combine,angleGrad,1,math.huge,timeoutDistance)
	--print("maxnumber Ccw = "..tostring(#tableCcw))
	local tableCw = courseplay:scanFruit(combine,angleGrad,-1, #tableCcw,timeoutDistance)
	if tableCw then
		outputTable = tableCw
	else
		outputTable = tableCcw
	end
	return outputTable
end

function courseplay:scanFruit(combine,startAngle , direction, maxPoints,timeoutDistance)
	local tempTable ={}
	local angleGrad = startAngle
	local cx,cy,cz =  getWorldTranslation(combine.cp.DirectionNode);
	for i= 1,359 do
		print("i= "..i)
		if i>= maxPoints then
			--print("return ")
			return nil
		end
		
		angleGrad = angleGrad + (1*direction)
		if angleGrad > 360 then
			angleGrad = angleGrad -360
		end
		--print("new angleGrad = "..tostring(angleGrad))
		
		ldX, ldZ = Utils.getDirectionFromYRotation(math.rad(angleGrad))
		--print("new local ldX: "..tostring(ldX).." ldZ: "..tostring(ldZ))
		local wdX,wdY,wdZ = localDirectionToWorld(combine.cp.DirectionNode, ldX, ldY,ldZ)
		--print("world wdX: "..tostring(wdX).." wdZ: "..tostring(wdZ))
		local foundFruit, distance = courseplay:getFieldBorderInDirection(combine,wdX,wdZ,fruitType,timeoutDistance)
		if not foundFruit then
			--print("stop searching")
			break
		end
		--print("distance = "..tostring(distance))
		local icx = cx+(wdX*(distance+5))
		local icz = cz+(wdZ*(distance+5))
		table.insert(tempTable, { x = icx, z = icz });
	end	
	return tempTable
end	

function courseplay:getFieldBorderInDirection(combine,wdX,wdZ,fruitType,timeoutDistance)
	local safetyRadius = 5
	local cx,cy,cz =  getWorldTranslation(combine.cp.DirectionNode);
	local firstHit = false
	for i=safetyRadius,timeoutDistance do
		local x = cx+(wdX*i)
		local z = cz+(wdZ*i)
		if not courseplay:areaHasFruit(x, z, fruitType, 1, 1) then
			if firstHit then
				return true,i
			end
			if i == timeoutDistance then
				return false
			
			end
		else
			firstHit = true
		end
	end	
end