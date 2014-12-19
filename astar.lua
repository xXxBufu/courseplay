local abs, max, min, pow, sin , huge = math.abs, math.max, math.min, math.pow, math.sin, math.huge;

function courseplay:calcMoves(vehicle,tx,tz,fruitType) 
	print(string.format("calcMoves(vehicle,tx(%s),tz(%s), fruitType%s) ",tostring(tx),tostring(tz),tostring(fruitType)))
	
	local sx,sy,sz =  getWorldTranslation(vehicle.cp.DirectionNode);
	local _,ty,_ = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, tx, 0, tz) 
	local tableCw = {}
	local tableCcw = {}
	local outputTable ={}
	local travelDistanceCcw,travelDistanceCw = 0,0
	
	
	targetNode = createTransformGroup('cpFruitFindingTargetNode');
	link(getRootNode(), targetNode);
	--local x,y,z = getTranslation(targetNode)
	setTranslation(targetNode, tx, ty, tz);

	
	local startDistance = courseplay:distance(tx,tz,sx,sz)
	local ldX, ldY,ldZ = courseplay:getDriveDirection(targetNode, sx,sy,sz)
	
	local timeoutDistance = startDistance * 2
	local angleGrad = math.deg(Utils.getYRotationFromDirection(ldX, ldZ))
	--print("angleGrad = "..tostring(angleGrad))
	--print("call Ccw")
	tableCcw, travelDistanceCcw = courseplay:scanFruit(vehicle,targetNode,angleGrad,1,fruitType,huge,timeoutDistance,startDistance)
	print("travelDistanceCcw = "..tostring(travelDistanceCcw))
	--print("call Cw")
	tableCw , travelDistanceCw  = courseplay:scanFruit(vehicle,targetNode,angleGrad,-1,fruitType, travelDistanceCcw,timeoutDistance,startDistance)
	print("travelDistanceCw = "..tostring(travelDistanceCw))
	if travelDistanceCw < travelDistanceCcw then
		outputTable = tableCw
	else
		outputTable = tableCcw
	end
		
	unlink(targetNode);
	delete(targetNode);
	
	return outputTable
end

function courseplay:scanFruit(vehicle,targetNode,startAngle,direction,fruitType,maxLastDistance,timeoutDistance,startDistance)
	local tempTable ={}
	local distance = startDistance   
	local totalDistance = 0
	local foundFruit = false
	local angleGrad = startAngle
	local backUpAngle = 0
	local cx,cy,cz = getWorldTranslation(targetNode);
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
		local wdX,wdY,wdZ = localDirectionToWorld(targetNode, ldX, 0,ldZ)
		local lastDistance = distance
		foundFruit, distance = courseplay:getFieldBorderInDirection(targetNode,wdX,wdZ,fruitType,distance,timeoutDistance)
		if not foundFruit then -- backup if smart fining doesn't find st
			--print(string.format("1st checking no fruit found"))
			foundFruit, distance = courseplay:getFieldBorderInDirection(targetNode,wdX,wdZ,fruitType,0,timeoutDistance)
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
		local loopWatch =  courseplay.utils:get​NumIsWithinTolerance​(math.ceil(backUpAngle),math.ceil(angleGrad),2) --was the last 2nd check at the same angle or near?
		
		if lastDistance > distance+30 and not loopWatch then
			--print(string.format("lastDistance (%d) > distance+50(%d)	jump found at %d Grad", lastDistance ,distance,angleGrad))
			local secondAngleGrad = angleGrad +(2*-direction)
			--print(string.format("	set angle to %s Grad",tostring(secondAngleGrad)))
			local goBack = true
			local secondFoundFruit =false
			local secondDistance = distance
			backUpAngle = angleGrad
			while goBack do
				i= i-1
				--print(string.format("	2nd checking %d Grad, distance: %d, fruitType: %s",secondAngleGrad,secondDistance,tostring(fruitType)))
				local sldX, sldZ = Utils.getDirectionFromYRotation(math.rad(secondAngleGrad))
				local swdX,swdY,swdZ = localDirectionToWorld(targetNode, sldX, 0,sldZ)
				secondFoundFruit, distance = courseplay:getFieldBorderInDirection(targetNode,swdX,swdZ,fruitType,secondDistance,10,true)
				
				if not secondFoundFruit then
					--print("	fruit is full, go back to normal")
					angleGrad = secondAngleGrad + (2*direction)
					distance = secondDistance
					--print("distance = "..tostring(distance))
					goBack = false
				else
					--print(string.format("	2nd checking new distance: %d",distance))
					secondDistance = distance 
					
					local icx = cx+(swdX*(distance-5))
					local icz = cz+(swdZ*(distance-5))
					
					
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

function courseplay:getFieldBorderInDirection(targetNode,wdX,wdZ,fruitType,distance,timeoutDistance,inversed)
	local safetyRadius = 5
	local forward = 1
	if inversed then
		safetyRadius = max(distance+50,safetyRadius)
		forward = -1
	else
		safetyRadius = max(distance-50,safetyRadius)
	end
	local cx,cy,cz =  getWorldTranslation(targetNode);
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

