local abs, max, min, pow, sin , huge = math.abs, math.max, math.min, math.pow, math.sin, math.huge;

function courseplay:calcMoves(vehicle,fruitType) 
	--print(string.format("calcMoves(vehicle, fruitType%s) ",tostring(fruitType)))
	local combine = vehicle.cp.activeCombine
	local sx,sy,sz =  getWorldTranslation(vehicle.cp.DirectionNode);
	local tableCw = {}
	local tableCcw = {}
	local outputTable ={}
	local travelDistanceCcw,travelDistanceCw = 0,0
	local startDistance = courseplay:distanceToPoint(combine, sx,sy,sz)
	local ldX, ldY,ldZ = courseplay:getDriveDirection(combine.cp.DirectionNode, sx,sy,sz)
	local timeoutDistance = courseplay:distanceToPoint(combine, sx,sy,sz) * 2
	local angleGrad = math.deg(Utils.getYRotationFromDirection(ldX, ldZ))
	--print("angleGrad = "..tostring(angleGrad))
	--print("call Ccw")
	tableCcw, travelDistanceCcw = courseplay:scanFruit(combine,angleGrad,1,fruitType,huge,timeoutDistance,startDistance)
	print("travelDistanceCcw = "..tostring(travelDistanceCcw))
	--print("call Cw")
	tableCw , travelDistanceCw  = courseplay:scanFruit(combine,angleGrad,-1,fruitType, travelDistanceCcw,timeoutDistance,startDistance)
	print("travelDistanceCw = "..tostring(travelDistanceCw))
	if travelDistanceCw < travelDistanceCcw then
		outputTable = tableCw
	else
		outputTable = tableCcw
	end
	return outputTable
end

function courseplay:scanFruit(combine,startAngle,direction,fruitType,maxLastDistance,timeoutDistance,startDistance)
	local tempTable ={}
	local distance = startDistance   
	local totalDistance = 0
	local foundFruit = false
	local angleGrad = startAngle
	local backUpAngle = 0
	local cx,cy,cz =  getWorldTranslation(combine.cp.DirectionNode);
	local i = 1
	while i<= 360 do
		i= i+1	
		angleGrad = angleGrad + (1*direction)
		if angleGrad > 360 then
			angleGrad = angleGrad -360
		elseif angleGrad < 0 then 
			angleGrad = angleGrad +360
		end
		--print(string.format("1st checking %d Grad distance: %d, fruitType: %s",angleGrad,distance,tostring(fruitType)))
		local ldX, ldZ = Utils.getDirectionFromYRotation(math.rad(angleGrad))
		local wdX,wdY,wdZ = localDirectionToWorld(combine.cp.DirectionNode, ldX, 0,ldZ)
		local lastDistance = distance
		foundFruit, distance = courseplay:getFieldBorderInDirection(combine,wdX,wdZ,fruitType,distance,timeoutDistance)
		if not foundFruit then -- backup if smart fining doesn't find st
			--print(string.format("1st checking no fruit found"))
			foundFruit, distance = courseplay:getFieldBorderInDirection(combine,wdX,wdZ,fruitType,0,timeoutDistance)
			--print(string.format("1st checking no fruit found new distance: %s",tostring(distance)))
			if not foundFruit then --still nothing found
				--print(string.format("1st checking backup no fruit found -> break"))
				break
			end
		else
			timeoutDistance = distance * 3
			--print(string.format("1st checking new distance: %s",tostring(distance)))
		end
		--is there a jump in distances?
									--50
		if lastDistance > distance+30 and math.ceil(angleGrad) ~= math.ceil(backUpAngle) then
			--print(string.format("lastDistance (%d) > distance+50(%d)	jump found at %d Grad", lastDistance ,distance,angleGrad))
			local secondAngleGrad = angleGrad +(2*-direction)
			--print(string.format("	set angle to %s Grad",tostring(secondAngleGrad)))
			local goBack = true
			local secondFoundFruit =false
			local secondDistance = lastDistance
			backUpAngle = angleGrad
			while goBack do
				i= i-1
				--print(string.format("	2nd checking %d Grad, distance: %d, fruitType: %s",secondAngleGrad,secondDistance,tostring(fruitType)))
				local sldX, sldZ = Utils.getDirectionFromYRotation(math.rad(secondAngleGrad))
				local swdX,swdY,swdZ = localDirectionToWorld(combine.cp.DirectionNode, sldX, 0,sldZ)
				secondFoundFruit, distance = courseplay:getFieldBorderInDirection(combine,swdX,swdZ,fruitType,secondDistance,10,true)
				
				if not secondFoundFruit then
					--print("	fruit is full, go back to normal")
					angleGrad = secondAngleGrad + (2*direction)
					distance = secondDistance
					--print("distance = "..tostring(distance))
					goBack = false
				else
					--print(string.format("	2nd checking new distance: %d",distance))
					secondDistance = distance 
									-- put new way point to table
									local icx = cx+(swdX*(distance-5))
									local icz = cz+(swdZ*(distance-5))
											
									-- calculate the travel distance of the course
									if #tempTable>1 then
										totalDistance = totalDistance + courseplay:distance(icx, icz, tempTable[#tempTable].x, tempTable[#tempTable].z)
									end
									if totalDistance > maxLastDistance then
										return tempTable , totalDistance
									end		
									table.insert(tempTable, { x = icx, z = icz });
					secondAngleGrad = secondAngleGrad + (1*-direction)
					if secondAngleGrad > 360 then
						secondAngleGrad = secondAngleGrad -360
					elseif secondAngleGrad < 0 then 
							secondAngleGrad = secondAngleGrad +360
					end
				end
			end			
			--return tempTable , totalDistance
		else	
				-- put new way point to table
				local icx = cx+(wdX*(distance+5))
				local icz = cz+(wdZ*(distance+5))
						
				-- calculate the travel distance of the course
				if #tempTable>1 then
					totalDistance = totalDistance + courseplay:distance(icx, icz, tempTable[#tempTable].x, tempTable[#tempTable].z)
				end
				if totalDistance > maxLastDistance then
					return tempTable , totalDistance
				end		
				table.insert(tempTable, { x = icx, z = icz });
		end
	end	
	return tempTable , totalDistance
end	

function courseplay:getFieldBorderInDirection(combine,wdX,wdZ,fruitType,distance,timeoutDistance,inversed)
	local safetyRadius = 5
	local forward = 1
	if inversed then
		safetyRadius = max(distance+50,safetyRadius)
		forward = -1
	else
		safetyRadius = max(distance-50,safetyRadius)
	end
	local cx,cy,cz =  getWorldTranslation(combine.cp.DirectionNode);
	local firstHit = false
	--print("	check distance = "..safetyRadius)
	for i=safetyRadius,timeoutDistance,forward do
		local x = cx+(wdX*i)
		local z = cz+(wdZ*i)
		if not courseplay:areaHasFruit(x, z, fruitType, 2, 2) then
			if firstHit then
				--if inversed then print("	return true, "..tostring(i)) end
				return true,i
			end
		else
			--print("		Hit = "..i)
			firstHit = true
		end
	end
	return false
end

