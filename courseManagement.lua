-- FS15 JAKOB
-- saving // loading courses
local curFile = 'courseManagement.lua';
local huge, max, random = math.huge, math.max, math.random;

function courseplay.courses:setup()
	if g_currentMission.courseplay == nil then
		g_currentMission.courseplay = {};
	end;

	g_currentMission.courseplay.courses = {};
	g_currentMission.courseplay.folders = {};
	g_currentMission.courseplay.sorted = {
		item = {},
		info = {}
	};

	-- TODO: TEMP OLD -> delete when done;
	if g_currentMission.cp_courses == nil then
		--courseplay:debug("cp courses was nil and initialized", 8);
		g_currentMission.cp_courses = {};
		g_currentMission.cp_folders = {};
		g_currentMission.cp_sorted = {item={}, info={}};
	end;

	self.courseReferenceNodeIndexToCourse = {};
	self.maxCourseReferenceNodeIndex = -1;
end;

function courseplay.courses:getCourseFileName(courseId)
	-- print(('getCourseFileName(%q)'):format(tostring(courseId)));
	local course = g_currentMission.courseplay.courses[courseId];
	assert(course ~= nil, ('ERROR: getCourseFileName(%s): course doesn\'t exist'):format(tostring(courseId)));

	if course.fileName then
		return course.fileName:sub(1, courseFileName:len() - 4);
	end;

	if course.name == nil then
		local ret = '_NoName_' .. random(1,200);
		print(('getCourseFileName(%q)'):format(tostring(course.name)));
		print(('\tnil -> return %q'):format(tostring(ret)));
		return ret;
	end;

	local fileName = courseplay:normalizeUTF8(course.name, true);
	-- print(('\tnormalizeUTF8() -> courseName=%q'):format(tostring(courseName)));

	return fileName;
end;

function courseplay.courses:getCourseXmlFile(courseId)
	local course = g_currentMission.courseplay.courses[courseId];
	if not course then
		return nil;
	end;

	local fileName = self:getCourseFileName(courseId) .. '.xml';
	local filePath = courseplay.cpSavegameFolderPath .. fileName;
	-- print(('getCourseXmlFile(%d): fileName=%q, filePath=%q'):format(courseId, tostring(fileName), tostring(filePath)));
	local file;
	if fileExists(filePath) then
		file = loadXMLFile('tempCourseFile', filePath);
	else
		file = createXMLFile('tempCourseFile', filePath, 'XML');
	end;

	if file == nil or file == 0 then
		return nil;
	end;

	return file, fileName;
end;

function courseplay.courses:saveCourseReferenceToXml(courseId, fileName)
	local course = g_currentMission.courseplay.courses[courseId];

	local cpFile, deleteCpFile = nil, false;
	if courseplay.cpXmlFileTemp and courseplay.cpXmlFileTemp ~= 0 then
		cpFile = courseplay.cpXmlFileTemp;
	else
		cpFile = loadXMLFile('cpXmlFileTemp', courseplay.cpXmlFilePath);
		deleteCpFile = true;
	end;

	if not cpFile or cpFile == 0 or not course then
		print('ERROR: saveCourseReferenceToXml(): cpFile=nil ... TODO');
		return;
	end;

	if fileName == nil then
		fileName = self:getCourseFileName(courseId) .. '.xml';
	end;

	--[[ TODO
	<courseReference>
		<course id="162" name="asdf->bläh" filename="asdf-_blaeh.xml" parent="0" numWaypoints="34" />
			--> mit speichern des node-indexes in tabelle beim laden
			--> course.courseReferenceNodeIndex = 2;
			--> courseplay.courses.courseReferenceNodeIndexToCourse[index] = course;
	</courseReference>
	]]

	if not course.courseReferenceNodeIndex then
		self:createNewCourseReferenceNodeIndex(course);
	end;

	local node = ('XML.courseReference.course(%d)'):format(course.courseReferenceNodeIndex);
	setXMLString(cpFile, node .. '#name', course.name);
	setXMLString(cpFile, node .. '#filename', fileName);
	setXMLInt(cpFile, node .. '#id', course.id);
	setXMLInt(cpFile, node .. '#parent', course.parent);
	setXMLInt(cpFile, node .. '#numWaypoints', #course.waypoints);

	saveXMLFile(cpFile);

	if deleteCpFile then
		delete(cpFile);
	end;
end;

function courseplay.courses:saveCourseToXml(courseId)
	local file, fileName = self:getCourseXmlFile(courseId);
	assert(file ~= nil, ('ERROR: saveCourseToXml(%d): file (%q) couldn\'t be loaded or created (is nil)'):format(courseId, fileName));
	assert(file ~= 0, ('ERROR: saveCourseToXml(%d): file (%q) couldn\'t be loaded or created'):format(courseId, fileName));

	local course = g_currentMission.courseplay.courses[courseId];
	assert(file ~= 0, ('ERROR: saveCourseToXml(%d): course doesn\'t exist'):format(courseId));

	print(('saveCourseToXml(%d): file=%q, fileName=%q, course=%q'):format(courseId, tostring(file), tostring(fileName), tostring(course)));

	-- reference in _courseplay.xml
	self:saveCourseReferenceToXml(courseId, fileName);

	----------------------------------------------------------------------------------------------------

	-- the course file
	local node = 'XML.course';
	setXMLString(file, node .. '#name', course.name);
	setXMLInt(file, node .. '#id', course.id);
	setXMLInt(file, node .. '#parent', course.parent);
	setXMLInt(file, node .. '#numWaypoints', #course.waypoints);

	local wp;
	for i=1, #course.waypoints do
		node = 'XML.course.waypoint' .. i;
		wp = course.waypoints[i];

		local pos = ('%.4f %.4f'):format(wp.cx, wp.cz);
		setXMLString(file, node .. '#pos', pos);

		setXMLString(file, node .. '#angle', ('%.2f'):format(wp.angle or 0));

		setXMLInt(file, node .. '#wait',	 Utils.getNoNil(courseplay:boolToInt(wp.wait), 0));
		setXMLInt(file, node .. '#crossing', Utils.getNoNil(courseplay:boolToInt(wp.crossing), 0));
		setXMLInt(file, node .. '#rev',		 Utils.getNoNil(courseplay:boolToInt(wp.rev), 0));

		setXMLString(file, node .. '#speed', ('%.5f'):format(wp.speed or 0));

		if wp.laneDir then
			setXMLString(file, node .. '#dir', tostring(wp.laneDir));
		end;
		setXMLString(file, node .. '#turn',		 tostring(Utils.getNoNil(wp.turn, false)));
		setXMLInt(file, node .. '#turnstart',	 courseplay:boolToInt(wp.turnStart) or 0);
		setXMLInt(file, node .. '#turnend',		 courseplay:boolToInt(wp.turnEnd) or 0);
		if wp.ridgeMarker then
			setXMLInt(file, node .. '#ridgemarker', wp.ridgeMarker);
		end;
		-- setXMLString(file, node .. '#generated', tostring(Utils.getNoNil(wp.generated, false))); -- TODO: needed?
	end;

	self:saveAndCloseXmlFile(file);
end;

function courseplay.courses:loadInitialData()
	assert(fileExists(courseplay.cpXmlFilePath), ('ERROR: courseplay file doesn\'t exist at %q'):format(courseplay.cpXmlFilePath));

	-- STEP 1: create data from existing courseReference
	self:loadCourseReferences()
end;

function courseplay.courses:loadCourseReferences()
	assert(fileExists(courseplay.cpXmlFilePath), ('ERROR: courseplay file doesn\'t exist at %q'):format(courseplay.cpXmlFilePath));

	-- STEP 1: create data from existing courseReference
	local cpFile = loadXMLFile('cpXmlFileTemp', courseplay.cpXmlFilePath);
	assert(cpFile and cpFile ~= 0, 'ERROR: loadCourseReferences(): cpFile=nil ... TODO');

	local i = 0;
	self.courses = {};
	self.fileNameToCourse = {};
	local maxId = 1;
	while true do
		local node = ('XML.courseReference.course(%d)'):format(i);
		local id = getXMLInt(cpFile, node .. '#id');
		if id == nil then
			break;
		end;

		local course = {
			id = id;
			uid = 'c' .. id;
			name =		 getXMLString(cpFile, node .. '#name');
			fileName =	 getXMLString(cpFile, node .. '#filename'); -- TODO: make sure fileName ~= nil;
			parent =		getXMLInt(cpFile, node .. '#parent');
			numWaypoints =	getXMLInt(cpFile, node .. '#numWaypoints');
			hasWaypointData = false;
		};
		assert(course.fileName ~= nil, ('ERROR: fileName doesn\'t exist for course %q'):format(course.name));
		local filePath = courseplay.cpSavegameFolderPath .. course.fileName;
		assert(fileExists(filePath), ('ERROR: file %q doesn\'t exist'):format(course.fileName));
		self:setCourseReferenceNodeIndex(course, i);
		self:addCourseToTables(course);
		maxId = max(maxId, id);

		i = i + 1;
	end;



	-- STEP 2: get new course files and add to courseReference
	local newCourses = {};
	local files = Files:new(courseplay.cpSavegameFolderPath);
	for k,file in ipairs(files.files) do
		print(('file.filename=%q, isCourseplayXml=%s, already in ref table=%s'):format(tostring(file.filename), tostring(file.filename == '_courseplay.xml'), tostring(self.fileNameToCourse[file.filename] ~= nil)));
		if file.isDirectory or file.filename == '_courseplay.xml' or self.fileNameToCourse[file.filename] ~= nil then
			-- SKIP...
		elseif Utils.endsWith(file.filename, '.xml') then
			print(('\tadd to ref table'));
			local path = courseplay.cpSavegameFolderPath .. file.filename;
			local courseFile = loadXMLFile('tmp', path);
			if courseFile and courseFile ~= 0 then
				local node = 'XML.course';
				if hasXMLProperty(courseFile, node) then
					print(('\tcourse file loaded at %q'):format(tostring(path)));
					local course = {
						id = maxId + 1;
						uid = 'c' .. maxId + 1;
						name =  getXMLString(courseFile, node .. '#name');
						fileName = file.filename;
						parent = 0; -- parent = getXMLInt(courseFile, node .. '#parent'); -- TODO: will this work?
						numWaypoints = getXMLInt(courseFile, node .. '#numWaypoints');
						hasWaypointData = false;
					};
					maxId = maxId + 1;
					self:createNewCourseReferenceNodeIndex(course);
					self:addCourseToTables(course);

					-- add course reference to _courseplay.xml
					-- TODO: make sure all values exist before setting them in cp file
					node = ('XML.courseReference.course(%d)'):format(course.courseReferenceNodeIndex);
					   setXMLInt(cpFile, node .. '#id',			  course.id);
					setXMLString(cpFile, node .. '#name',		  course.name);
					setXMLString(cpFile, node .. '#filename',	  course.fileName);
					   setXMLInt(cpFile, node .. '#parent',		  course.parent);
					   setXMLInt(cpFile, node .. '#numWaypoints', course.numWaypoints);

					i = i + 1;
				end;
			end;
			delete(courseFile);
		end;
	end;

	self:saveAndCloseXmlFile(cpFile);

	print(tableShow(self.courses, 'courseplay.courses.courses'));
end;

function courseplay.courses:addCourseToTables(course)
	self.courses[#self.courses + 1] = course;
	self.fileNameToCourse[course.fileName] = course;
	g_currentMission.courseplay.courses[course.id] = course;
end;

-- TODO (Jakob): when deleting a course, not only do we have to delete the index and table reference, but we also have to update all courseReferences that come after the deleted course
-- TODO (Jakob): we're gonna have to make sure that courses don't have identical names, or even fileNames
	--> "a:sdf" vs "asdf" is okay, as "a_sdf.xml" vs "asdf.xml"
	--> "a:sdf" vs "a_sdf" is not okay, as "a_sdf.xml" vs "a_sdf.xml"
	--> create courseplay.courses.courseNameToCourse[courseName] (or [courseFileName]) -> when saving course in inputCourseNameDialogue (or even on every character change), check table, alert if duplicate name
function courseplay.courses:setCourseReferenceNodeIndex(course, index)
	assert(self.courseReferenceNodeIndexToCourse[index] == nil, ('ERROR: (course %q): another course with than nodeIndex (%d) already exists'):format(course.name, index));

	self.maxCourseReferenceNodeIndex = max(self.maxCourseReferenceNodeIndex, index);
	course.courseReferenceNodeIndex = index;
	self.courseReferenceNodeIndexToCourse[index] = course;

	print(('setCourseReferenceNodeIndex(%q); courseReferenceNodeIndex=%d'):format(course.name, course.courseReferenceNodeIndex));
end;

function courseplay.courses:createNewCourseReferenceNodeIndex(course)
	self.maxCourseReferenceNodeIndex = self.maxCourseReferenceNodeIndex + 1;
	course.courseReferenceNodeIndex = courseplay.courses.maxCourseReferenceNodeIndex;
	self.courseReferenceNodeIndexToCourse[course.courseReferenceNodeIndex] = course;

	print(('createNewCourseReferenceNodeIndex(%q); courseReferenceNodeIndex=%d'):format(course.name, course.courseReferenceNodeIndex));
end;

function courseplay.courses:refreshAll()
	g_currentMission.courseplay.sorted = courseplay.courses.sort()
	courseplay.settings.setReloadCourseItems()
	courseplay.utils.signs:updateWaypointSigns(vehicle);
end;

function courseplay.courses:deleteSortedItem(vehicle, index)
	local id = vehicle.cp.hud.courses[index].id;
	local type = vehicle.cp.hud.courses[index].type;

	local changed = false;
	if type == 'course' then
		changed = self:deleteCourse(id);
	elseif type == 'folder' then
		if self:getFolderHasChildren(id) then -- check for children: delete only if folder has no children
			return;
		end;
		changed = self:deleteFolder(id);
	else
		-- ERROR
	end;

	--[[ TODO
	if changed then
		self:refreshAll();
	end;
	]]
end;

function courseplay.courses:deleteFolder(folderId)
	g_currentMission.courseplay.folders[id] = nil;

	local cpFile = loadXMLFile('cpXmlFileTemp', courseplay.cpXmlFilePath);
	assert(cpFile and cpFile ~= 0, 'ERROR: deleteCourse(): _courseplay.xml couldn\'t be opened');

	local changed = false;
	-- TODO: use nodeIndex reference system as with courses ?
	local i = 0;
	while true do
		local node = ('XML.folders.folder(%d)'):format(i);
		if not hasXMLProperty(cpFile, node) then
			break;
		end;

		local folderId = getXMLInt(cpFile, node .. '#id');
		if folderId and folderId == id then
			removeXMLProperty(cpFile, node);
			changed = true;
			break;
		end;

		i = i + 1;
	end;

	if changed then
		saveXMLFile(cpFile);
	end;
	delete(cpFile);

	return changed;
end;

function courseplay.courses:getFolderHasChildren(folderId)
	return g_currentMission.courseplay.sorted.info['f' .. folderId].lastChild ~= 0;
end;

-- TODO: test deleteCourse()
function courseplay.courses:deleteCourse(courseId)
	local cpFile, closeCpFile = nil, false;
	if courseplay.cpXmlFileTemp and courseplay.cpXmlFileTemp ~= 0 then
		cpFile = courseplay.cpXmlFileTemp;
	else
		cpFile = loadXMLFile('cpXmlFileTemp', courseplay.cpXmlFilePath);
		deleteCpFile = true;
	end;
	assert(cpFile and cpFile ~= 0, 'ERROR: deleteCourse(): _courseplay.xml couldn\'t be opened');

	local course = g_currentMission.courseplay.courses[courseId];
	local fileName = course.fileName;
	local courseReferenceNodeIndex = course.courseReferenceNodeIndex;

	local filePath = courseplay.cpSavegameFolderPath .. fileName;
	assert(fileExists(filePath), ('ERROR: deleteCourse(%d): course file for course %q doesn\'t exist at %q'):format(courseId, course.name, filePath));
	deleteFile(filePath);

	local node = ('XML.courseReference.course(%d)'):format(courseReferenceNodeIndex);
	removeXMLProperty(cpFile, node);
	saveXMLFile(cpFile);
	if closeCpFile then
		delete(cpFile);
	end;

	g_currentMission.courseplay.courses[courseId] = nil;
	self.courseReferenceNodeIndexToCourse[courseReferenceNodeIndex] = nil;
	self.fileNameToCourse[fileName];
	course = nil;

	self:resetCourseReferenceNodeIndexes(courseReferenceNodeIndex + 1);
	-- TODO: reset courses, re-sort, update hud etc.
	-- self:refreshAll();

	return true;
end;

function courseplay.courses:resetCourseReferenceNodeIndexes(startIndex)
	startIndex = startIndex or 0;

	local maxIndex = self.maxCourseReferenceNodeIndex; -- use temp var because maxCourseReferenceNodeIndex will be changed in the loop
	for i=startIndex, maxIndex do
		local course = self.courseReferenceNodeIndexToCourse[i];
		if not course then
			print('ERROR: course not found at nodeIndex ' .. i);
			-- break;
		else
			self:setCourseReferenceNodeIndex(course, i - 1);
		end;
	end;

	self.maxCourseReferenceNodeIndex = table.maxn(self.courseReferenceNodeIndexToCourse);
end;

function courseplay.courses:loadWaypointsFromXml(courseId)
	-- only to be called if course.hasWaypointData ~= true -> course waypoints can be reloaded from xml when user clicks "reload" button
	local course = g_currentMission.courseplay.courses[courseId];
	if not course then
		-- ERROR 1: course doesn't exist
		return false;
	end;

	if course.hasWaypointData then
		-- ERROR 2: already has waypoints
		return false;
	end;

	local filePath = courseplay.cpSavegameFolderPath .. course.fileName;
	if not fileExists(filePath) then
		-- ERROR 3: course file can't be found
		return false;
	end;

	local file = loadXMLFile(filePath);
	if not file or file == 0 then
		-- ERROR 4: course file can't be read
		return false;
	end;

	course.waypoints = {};
	local i = 1;
	local node;
	while true do
		node = ('XML.course.waypoint%d'):format(i);
		if not hasXMLProperty(file, node) then
			break;
		end;

		local x, z = Utils.getVectorFromString(getXMLString(file, node .. '#pos'));
		course.waypoints[i] = {
			cx =			x;
			cz =			z;
			angle =			Utils.getVectorFromString(getXMLString(file, node .. '#angle'));
			wait =			courseplay:intToBool(getXMLInt(file, node .. '#wait'));
			speed =			Utils.getVectorFromString(getXMLString(file, node .. '#speed'));
			rev =			courseplay:intToBool(getXMLInt(file, node .. '#rev'));
			crossing =		courseplay:intToBool(getXMLInt(file, node .. '#crossing'));
			-- generated =	getXMLBool(file, node .. '#generated') or false; -- TODO: needed?
			dir =		 getXMLString(file, node .. '#dir');
			turn =		 getXMLString(file, node .. '#turn') or false;
			turnStart =		getXMLInt(file, node .. '#turnstart') or 0;
			turnEnd =		getXMLInt(file, node .. '#turnend') or 0;
			ridgeMarker =	getXMLInt(file, node .. '#ridgemarker') or 0;
		};

		i = i + 1;
	end;

	if course.numWaypoints ~= #course.waypoints then
		course.numWaypoints = #course.waypoints;
		setXMLInt(file, 'XML.course#numWaypoints', course.numWaypoints);
	end;
	course.hasWaypointData = course.numWaypoints ~= 0;

	delete(file);
end;

function courseplay.courses:resetMergedWaypoints()
	for _,course in pairs(g_currentMission.courseplay.courses) do
		for _,wp in ipairs(course.waypoints) do
			wp.merged = nil;
		end;
	end;
end;

function courseplay.courses:saveParentToXml(type, id)
	assert(id and id ~= 0, ('ERROR: saveParent(%s, ...): id not valid'):format(tostring(type);

	local node, value;
	if type == 'course' then
		local course = g_currentMission.courseplay.courses[id];
		node = ('XML.courseReference.course(%d)'):format(course.courseReferenceNodeIndex);
		value = course.parent;
		if node and value then
			local file, fileName = self:getCourseXmlFile(id);
			if file then
				setXMLInt(file, 'XML.course#parent', value);
				self:saveAndCloseXmlFile(file);
			end;
		end;

	elseif type == 'folder' then
		local folder = g_currentMission.courseplay.folders[id];
		node = ('XML.folders.folder(%d)'):format(folder.folderNodeIndex);
		value = folder.parent;
	end;

	if node and value then
		local cpFile = loadXMLFile('cpXmlFileTemp', courseplay.cpXmlFilePath);
		setXMLInt(cpFile, node .. '#parent', value);
		self:saveAndCloseXmlFile(cpFile);
	end;
end;

function courseplay.courses:saveAndCloseXmlFile(file)
	saveXMLFile(file);
	delete(file);
end;