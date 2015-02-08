local abs, rad = math.abs, math.rad;

function courseplay:setupCourseGenerationPreviewData(vehicle)
	vehicle.cp.courseGenerationPreview.borderPoints = nil;
	vehicle.cp.startingPointCoordsManual = nil;

	local borderPoints = courseplay.fields.fieldData[vehicle.cp.fieldEdge.selectedField.fieldNum].simplePoly;

	vehicle.cp.courseGenerationPreview.dimensions = courseplay.utils:getCourseDimensions(borderPoints);
	local bBox = vehicle.cp.courseGenerationPreview.dimensions;
	local pxSize = 2;  -- thickness of line in pixels
	local height = pxSize / g_screenHeight;

	local bgPadding = 0.05 * bBox.span;
	local worldX1, worldX2 = bBox.xMin - bgPadding, bBox.xMax + bgPadding;
	local worldZ1, worldZ2 = bBox.yMin - bgPadding, bBox.yMax + bgPadding;
	local bgX1, bgY1 = courseplay.utils:worldCoordsTo2D(worldX1, worldZ1, vehicle.cp.courseGenerationPreview.dimensions);
	local bgX2, bgY2 = courseplay.utils:worldCoordsTo2D(worldX2, worldZ2, vehicle.cp.courseGenerationPreview.dimensions);
	local bgW, bgH = bgX2 - bgX1, abs(bgY2 - bgY1);

	vehicle.cp.courseGenerationPreview.background = {
		x = bgX1,
		y = bgY2, -- seems wrong, but is correct, as [3D] topZ < bottomZ, but [2D] topY > bottomY
		width = bgW,
		height = bgH,
		worldX1 = worldX1,
		worldX2 = worldX2,
		worldZ1 = worldZ2,
		worldZ2 = worldZ1, -- switched, as described before
		tractorVisAreaMinX = bgX1,
		tractorVisAreaMaxX = bgX2,
		tractorVisAreaMinY = bgY2,
		tractorVisAreaMaxY = bgY1
	};

	-- PDA MAP BG
	if vehicle.cp.courseGenerationPreview.pdaMapOverlay then
		local leftX	  = bBox.xMin - bgPadding + g_statisticView.worldCenterOffsetX;
		local bottomY = bBox.yMax + bgPadding + g_statisticView.worldCenterOffsetZ;
		local rightX  = bBox.xMax + bgPadding + g_statisticView.worldCenterOffsetX;
		local topY	  = bBox.yMin - bgPadding + g_statisticView.worldCenterOffsetZ;
		courseplay.utils:setOverlayUVsPx(vehicle.cp.courseGenerationPreview.pdaMapOverlay, { leftX, bottomY, rightX, topY }, g_statisticView.worldSizeX, g_statisticView.worldSizeZ);

		vehicle.cp.courseGenerationPreview.pdaMapOverlay:setPosition(vehicle.cp.courseGenerationPreview.background.x, vehicle.cp.courseGenerationPreview.background.y);
		vehicle.cp.courseGenerationPreview.pdaMapOverlay:setDimension(vehicle.cp.courseGenerationPreview.background.width, vehicle.cp.courseGenerationPreview.background.height);
	end;

	vehicle.cp.courseGenerationPreview.borderPoints = {};
	local numBorderPoints = #borderPoints;
	local np, startX, startY, endX, endY, dx, dz, dx2D, dy2D, width, rotation;
	local r, g, b = unpack(courseplay.utils:rgbToNormal(243, 48, 255));
	for i,wp in ipairs(borderPoints) do
		np = i < numBorderPoints and borderPoints[i + 1] or borderPoints[1];

		startX, startY = courseplay.utils:worldCoordsTo2D(wp.cx, wp.cz, vehicle.cp.courseGenerationPreview.dimensions);
		endX, endY	   = courseplay.utils:worldCoordsTo2D(np.cx, np.cz, vehicle.cp.courseGenerationPreview.dimensions);

		dx2D = endX - startX;
		dy2D = (endY - startY) / g_screenAspectRatio;
		width = Utils.vector2Length(dx2D, dy2D);

		dx = np.cx - wp.cx;
		dz = np.cz - wp.cz;
		rotation = Utils.getYRotationFromDirection(dx, dz) - math.pi * 0.5;

		vehicle.cp.courseGenerationPreview.borderPoints[i] = {
			x = startX;
			y = startY;
			width = width;
			height = height;
			rotation = rotation;
			color = { r, g, b, 1 };
		};
	end;

	vehicle.cp.courseGenerationPreview.updateDrawData = false;
end;

function courseplay:drawCourseGenerationPreview(vehicle)
	-- dynamically update the data (when drag + drop happens)
	if vehicle.cp.courseGenerationPreview.updateDrawData then
		print(('%s: courseGenerationPreview.updateDrawData==true -> call setupCourseGenerationPreviewData()'):format(nameNum(vehicle)));
		courseplay:setupCourseGenerationPreviewData(vehicle);
	end;

	if not vehicle.cp.courseGenerationPreview.borderPoints then
		return;
	end;

	-- background
	local bg = vehicle.cp.courseGenerationPreview.background;
	if vehicle.cp.courseGenerationPreview.pdaMapOverlay then
		vehicle.cp.courseGenerationPreview.pdaMapOverlay:render();
	else
		if not CpManager.course2dDragDropMouseDown then
			setOverlayColor(CpManager.course2dPolyOverlayId, 0,0,0,0.6);
		end;
		renderOverlay(CpManager.course2dPolyOverlayId, bg.x, bg.y, bg.width, bg.height);
	end;

	if CpManager.course2dDragDropMouseDown ~= nil then -- drag and drop mode -> only render background
		return;
	end;

	-- border
	local numPoints = #vehicle.cp.courseGenerationPreview.borderPoints;
	local r,g,b,a;
	for i,data in ipairs(vehicle.cp.courseGenerationPreview.borderPoints) do
		r,g,b,a = unpack(data.color);
		setOverlayColor(CpManager.course2dPolyOverlayId, r,g,b,a);

		setOverlayRotation(CpManager.course2dPolyOverlayId, data.rotation, 0, 0);

		renderOverlay(CpManager.course2dPolyOverlayId, data.x, data.y, data.width, data.height);
	end;
	setOverlayRotation(CpManager.course2dPolyOverlayId, 0, 0, 0); -- reset overlay rotation


	-- starting point
	if vehicle.cp.startingPointCoordsManual then
		-- print(('vehicle.cp.courseGenerationPreview.startingPointOverlay:render()'))
		local spOvl = vehicle.cp.courseGenerationPreview.startingPointOverlay;

		-- starting direction
		if not vehicle.cp.startingDirectionIsAuto then
			setOverlayColor(CpManager.course2dPolyOverlayId, 0,1,0,1);
			setOverlayRotation(CpManager.course2dPolyOverlayId, rad(90 - vehicle.cp.startingDirection), 0, 0);

			renderOverlay(CpManager.course2dPolyOverlayId, spOvl.x + spOvl.width * 0.5, spOvl.y + spOvl.height * 0.5 - 1/g_screenHeight, 50/g_screenWidth, 2/g_screenHeight);
		end;

		spOvl:render();
	end;
end;
