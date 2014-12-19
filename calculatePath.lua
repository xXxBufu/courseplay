local abs, max, min, pow, sin , huge = math.abs, math.max, math.min, math.pow, math.sin, math.huge;

function courseplay:calcMoves(vehicle,tx,tz,fruitType) 
	courseplay:debug(string.format("calcMoves(vehicle,tx(%d), tz(%d), fruitType %s) ",tx,tz,tostring(fruitType)),22)
	
	local sx,sy,sz =  getWorldTranslation(vehicle.cp.DirectionNode);
	local _,ty,_ = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, tx, 0, tz) 
	local tableCw = {}
	local tableCcw = {}
	local outputTable ={}
	local travelDistanceCcw,travelDistanceCw = 0,0
	
	
	targetNode = createTransformGroup('cpFruitFindingTargetNode');
	link(getRootNode(), targetNode);
	setTranslation(targetNode, tx, ty, tz);

	
	local startDistance = courseplay:distance(tx,tz,sx,sz)
	local ldX, ldY,ldZ = courseplay:getDriveDirection(targetNode, sx,sy,sz)
	
	local timeoutDistance = startDistance * 2
	local angleGrad = math.deg(Utils.getYRotationFromDirection(ldX, ldZ),22)
	--courseplay:debug("angleGrad = "..tostring(angleGrad),22)
	courseplay:debug("call clockwise",22)
	tableCw, travelDistanceCw = courseplay:scanFruit(vehicle,targetNode,angleGrad,1,fruitType,huge,timeoutDistance,startDistance)
	courseplay:debug("call counter clockwise",22)
	tableCcw , travelDistanceCcw  = courseplay:scanFruit(vehicle,targetNode,angleGrad,-1,fruitType, travelDistanceCw,timeoutDistance,startDistance)
	courseplay:debug("travelDistanceCw = "..tostring(travelDistanceCcw),22)
	courseplay:debug("travelDistanceCcw = "..tostring(travelDistanceCw),22)
	
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
		courseplay:debug(string.format("1st checking: %d Grad distance: %d, fruitType: %s",angleGrad,distance,tostring(fruitType)),22)
		local ldX, ldZ = Utils.getDirectionFromYRotation(math.rad(angleGrad))
		local wdX,wdY,wdZ = localDirectionToWorld(targetNode, ldX, 0,ldZ)
		local lastDistance = distance
		foundFruit, distance = courseplay:getFieldBorderInDirection(targetNode,wdX,wdZ,fruitType,distance,timeoutDistance)
		if not foundFruit then -- backup if smart fining doesn't find st
			courseplay:debug(string.format("1st checking: no fruit found"),22)
			foundFruit, distance = courseplay:getFieldBorderInDirection(targetNode,wdX,wdZ,fruitType,0,timeoutDistance)
			courseplay:debug(string.format("1st checking: no fruit found new distance: %s",tostring(distance)),22)
			if not foundFruit then --still nothing found
				courseplay:debug(string.format("1st checking: backup no fruit found -> break"),22)
				break
			end
		else
			timeoutDistance = distance * 3
			courseplay:debug(string.format("1st checking: found in distance: %d",distance),22)
		end
		--is there a jump in distances?
									--50
		local loopWatch =  courseplay.utils:get​NumIsWithinTolerance​(math.ceil(backUpAngle),math.ceil(angleGrad),2) --was the last 2nd check at the same angle or near?
		local developersStop = true
		if lastDistance > distance+30 and not loopWatch and not developersStop then
			courseplay:debug(string.format("lastDistance (%d) > distance+50(%d)	jump found at %d Grad", lastDistance ,distance,angleGrad),22)
			local secondAngleGrad = angleGrad +(2*-direction)
			--courseplay:debug(string.format("	set angle to %s Grad",tostring(secondAngleGrad)),22)
			local goBack = true
			local secondFoundFruit =false
			local secondDistance = distance
			backUpAngle = angleGrad
			while goBack do
				i= i-1
				courseplay:debug(string.format("	2nd checking: %d Grad, distance: %d, fruitType: %s",secondAngleGrad,secondDistance,tostring(fruitType)),22)
				local sldX, sldZ = Utils.getDirectionFromYRotation(math.rad(secondAngleGrad))
				local swdX,swdY,swdZ = localDirectionToWorld(targetNode, sldX, 0,sldZ)
				secondFoundFruit, distance = courseplay:getFieldBorderInDirection(targetNode,swdX,swdZ,fruitType,secondDistance,10,true)
				
				if not secondFoundFruit then
					courseplay:debug("	2nd checking: fruit is full, go back to normal",22)
					angleGrad = secondAngleGrad + (2*direction)
					distance = secondDistance
					courseplay:debug("distance = "..tostring(distance),22)
					goBack = false
				else
					courseplay:debug(string.format("	2nd checking: found in distance: %d",distance),22)
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
	--courseplay:debug("	check distance = "..safetyRadius,22)
	for i=safetyRadius,timeoutDistance,forward do
		local x = cx+(wdX*i)
		local z = cz+(wdZ*i)
		if not courseplay:areaHasFruit(x, z, fruitType, 2, 2) then
			if firstHit then
				--if inversed then courseplay:debug("	return true, "..tostring(i),22) end
				return true,i
			end
		else
			--courseplay:debug("		Hit = "..i,22)
			firstHit = true
		end
	end
	return false
end

