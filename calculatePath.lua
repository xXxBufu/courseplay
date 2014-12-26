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
	tableCw, travelDistanceCw = courseplay:scanFruit(vehicle,targetNode,angleGrad,-1,fruitType,huge,timeoutDistance,startDistance)
	courseplay:debug("call counter clockwise",22)
	tableCcw , travelDistanceCcw  = courseplay:scanFruit(vehicle,targetNode,angleGrad,1,fruitType, travelDistanceCw,timeoutDistance,startDistance)
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
	local borderFound,back,backFromBack
	local i = 1
	while i<= 360 do
		i= i+1	
		angleGrad = angleGrad + (1*direction)
		if angleGrad > 360 then
			angleGrad = angleGrad -360
		elseif angleGrad < 0 then 
			angleGrad = angleGrad +360
		end
		local lastDistance = distance
		courseplay:debug(string.format("1st checking: %d Grad distance: %d, fruitType: %s",angleGrad,distance,tostring(fruitType)),22)
		local ldX, ldZ = Utils.getDirectionFromYRotation(math.rad(angleGrad))
		local wdX,wdY,wdZ = localDirectionToWorld(targetNode, ldX, 0,ldZ)
		foundFruit, distance = courseplay:getFruitBorderInDirection(targetNode,wdX,wdZ,fruitType,distance,timeoutDistance)
		if not foundFruit then -- backup if smart finding doesn't find something
			courseplay:debug(string.format("1st checking: no fruit found"),22)
			foundFruit, distance = courseplay:getFruitBorderInDirection(targetNode,wdX,wdZ,fruitType,0,timeoutDistance)
			courseplay:debug(string.format("1st checking: no fruit found new distance: %s",tostring(distance)),22)
			if not foundFruit then --still nothing found
				courseplay:debug(string.format("1st checking: backup no fruit found -> break"),22)
				break
			end
		else
			timeoutDistance = distance * 3
			courseplay:debug(string.format("1st checking: found in distance: %d",distance),22)
		end
		local icx = cx+(wdX*(distance))
		local icz = cz+(wdZ*(distance))
		
		--is there a jump in distances?
		local jump = abs(lastDistance - distance)
		if jump > 30 and lastDistance > 0 and not backFromBack then
			local oldX, oldZ = cx, cz
			if #tempTable > 1 then
				oldX, oldZ = tempTable[#tempTable].x, tempTable[#tempTable].z
			end
			local isField = courseplay:isLineField(nil,oldX,oldZ,icx,icz);
			local hasFruit = courseplay:hasLineFruit(nil,oldX,oldZ,icx,icz);
			courseplay:debug(string.format("jump found:old %d vs new %d; isField: %s, hasFruit: %s ",lastDistance,distance,tostring(isField),tostring(hasFruit)),22)
			if isField and hasFruit then
				courseplay:debug(string.format("isField and hasFruit"),22)
			elseif isField and not hasFruit then
				courseplay:debug(string.format("isField and not hasFruit"),22)
			elseif not isField then
				courseplay:debug(string.format("not isField"),22)
				back = true
				distance = lastDistance
				while back do
					if angleGrad > 360 then
						angleGrad = angleGrad -360
					elseif angleGrad < 0 then 
						angleGrad = angleGrad +360
					end					
					ldX, ldZ = Utils.getDirectionFromYRotation(math.rad(angleGrad))
					wdX,wdY,wdZ = localDirectionToWorld(targetNode, ldX, 0,ldZ)
					courseplay:debug(string.format("border checking: %d Grad distance: %d, fruitType: %s",angleGrad,distance,tostring(fruitType)),22)
					borderFound , distance = courseplay:getFieldBorderInDirection(targetNode,wdX,wdZ,distance)
					if borderFound then
						courseplay:debug(string.format("border found at %d",distance ),22)
						tempTable , totalDistance = courseplay:addWaypointToTargets(tempTable,totalDistance,cx,cz,wdX,wdZ,distance)
						angleGrad = angleGrad + (1*-direction)
					else
						courseplay:debug(string.format("no border found"),22)
						back = false
						backFromBack = true
						timeoutDistance = distance+30
					end									
				end
			end			
		else	
			backFromBack = false
			-- no jump, put new waypoint to table
			tempTable , totalDistance = courseplay:addWaypointToTargets(tempTable,totalDistance,cx,cz,wdX,wdZ,distance)
			
			--[[if totalDistance > maxLastDistance then
				return tempTable , totalDistance
			end	]]
		end
	end	
	return tempTable,totalDistance
end	

function courseplay:getFruitBorderInDirection(targetNode,wdX,wdZ,fruitType,distance,timeoutDistance)
	local safetyRadius = max(distance-50,5)
	local cx,cy,cz =  getWorldTranslation(targetNode);
	local firstHit = false
	local lastDistance = 0 
	--courseplay:debug("	check distance = "..safetyRadius,22)
	for i=safetyRadius,timeoutDistance do
		local x = cx+(wdX*i)
		local z = cz+(wdZ*i)
		if not courseplay:areaHasFruit(x, z, fruitType, 2, 2) then
			if firstHit then
				--if inversed then courseplay:debug("	return true, "..tostring(i),22) end
				lastDistance = i
				firstHit = false
			end
		else
			--courseplay:debug("		Hit = "..i,22)
			firstHit = true
		end
	end
	if lastDistance > 0 then
		return true,lastDistance
	else
		return false
	end
end

function courseplay:getFieldBorderInDirection(targetNode,wdX,wdZ,distance)
	local cx,cy,cz = getWorldTranslation(targetNode)
	for i=distance+10,5,-1 do
		local x,z = cx+(wdX*i),cz+(wdZ*i)
		if not courseplay:isField(x, z, 1, 1) then
			return true, i
		end
	end
		return false,distance
end


function courseplay:addWaypointToTargets(tempTable,totalDistance,cx,cz,wdX,wdZ,distance)
	local icx = cx+(wdX*(distance+5))
	local icz = cz+(wdZ*(distance+5))

	-- calculate the travel distance of the course
	if #tempTable>1 then
		totalDistance = totalDistance + courseplay:distance(icx, icz, tempTable[#tempTable].x, tempTable[#tempTable].z)
	end					
	table.insert(tempTable, { x = icx, z = icz });
	
	return tempTable, totalDistance
end
