local abs, max, min, pow, sin , huge = math.abs, math.max, math.min, math.pow, math.sin, math.huge;

function courseplay:calcMoves(vehicle,fruitType) 
	print("call calcMoves()");
	local combine = vehicle.cp.activeCombine
	local cx,cy,cz =  getWorldTranslation(combine.cp.DirectionNode);
	print("actual x: "..tostring(cx).." z: "..tostring(cz))
	local sx,sy,sz =  getWorldTranslation(vehicle.cp.DirectionNode);
	local outputTable ={}
	local ldX, ldY,ldZ = courseplay:getDriveDirection(combine.cp.DirectionNode, sx,sy,sz)
	print("local wdX: "..tostring(ldX).." wdZ: "..tostring(ldZ))
	local timeoutDistance = courseplay:distanceToPoint(combine, sx,sy,sz) * 2
	local distance = 0
	local angleGrad = math.deg(Utils.getYRotationFromDirection(ldX, ldZ))
	print("angleGrad = "..tostring(angleGrad))
	
	--counter clockwise
	for i= 1,359 do
		print("i= "..i)
		angleGrad = angleGrad + 1
		if angleGrad > 360 then
			angleGrad = angleGrad -360
		end
		print("new angleGrad = "..tostring(angleGrad))
		ldX, ldZ = Utils.getDirectionFromYRotation(math.rad(angleGrad))
		print("new local ldX: "..tostring(ldX).." ldZ: "..tostring(ldZ))
		local wdX,wdY,wdZ = localDirectionToWorld(combine.cp.DirectionNode, ldX, ldY,ldZ)
		print("world wdX: "..tostring(wdX).." wdZ: "..tostring(wdZ))
		local foundFruit, distance = courseplay:getFieldBorderInDirection(combine,wdX,wdZ,fruitType,timeoutDistance)
		if not foundFruit then
			print("stop searching")
			break
		end
		print("distance = "..tostring(distance))
		local icx = cx+(wdX*(distance+5))
		local icz = cz+(wdZ*(distance+5))
		table.insert(outputTable, { x = icx, z = icz });
	end	
	
	
	return outputTable
end


function courseplay:getFieldBorderInDirection(combine,wdX,wdZ,fruitType,timeoutDistance)
	local safetyRadius = 10
	local cx,cy,cz =  getWorldTranslation(combine.cp.DirectionNode);
	for i=safetyRadius,timeoutDistance do
		--print("checking "..i)
		local x = cx+(wdX*i)
		local z = cz+(wdZ*i)
		if not courseplay:areaHasFruit(x, z, fruitType, 1, 1) then
			--print("return  "..i)
			if i == safetyRadius or i == timeoutDistance then
				return false
			else
				return true,i
			end
		else
			--print("hit  "..i)
		end
	end	
end