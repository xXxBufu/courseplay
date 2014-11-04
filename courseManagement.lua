-- FS15 JAKOB
-- saving // loading courses
local curFile = 'courseManagement.lua';
local huge, max, random = math.huge, math.max, math.random;

function courseplay.courses:setup()
	g_currentMission.cp_courses = {};
	self.courseReferenceNodeIndexToCourse = {};
	self.maxCourseReferenceNodeIndex = -1;
end;

function courseplay.courses:getCourseFileName(courseId)
	-- print(('getCourseFileName(%q)'):format(tostring(courseId)));
	local course = g_currentMission.cp_courses[courseId];
	assert(course ~= nil, ('ERROR: getCourseFileName(%s): course doesn\'t exist'):format(tostring(courseId)));

	if course.name == nil then
		local ret = '_NoName_' .. random(1,200);
		print(('getCourseFileName(%q)'):format(tostring(course.name)));
		print(('\tnil -> return %q'):format(tostring(ret)));
		return ret;
	end;

	local courseName = courseplay:normalizeUTF8(course.name, true);
	-- print(('\tnormalizeUTF8() -> courseName=%q'):format(tostring(courseName)));

	return courseName;
end;

function courseplay.courses:getCourseXmlFile(courseId)
	local course = g_currentMission.cp_courses[courseId];
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
	local course = g_currentMission.cp_courses[courseId];

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

	local course = g_currentMission.cp_courses[courseId];
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

	saveXMLFile(file);
	delete(file);
end;


function courseplay.courses:loadCourseReferences()
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
		else
			print(('\tadd to ref table'));
			local path = courseplay.cpSavegameFolderPath .. file.filename;
			local courseFile = loadXMLFile('tmp', path);
			if courseFile and courseFile ~= 0 then
				print(('\tcourse file loaded at %q'):format(tostring(path)));
				local node = 'XML.course';
				local course = {
					id = maxId + 1;
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
			delete(courseFile);
		end;
	end;

	saveXMLFile(cpFile);
	delete(cpFile);

	print(tableShow(self.courses, 'courseplay.courses.courses'));
end;

function courseplay.courses:addCourseToTables(course)
	self.courses[#self.courses + 1] = course;
	self.fileNameToCourse[course.fileName] = course;
	g_currentMission.cp_courses[course.id] = course;
end;

-- TODO (Jakob): when deleting a course, not only do we have to delete the index and table reference, but we also have to update all courseReferences that come after the deleted course
-- TODO (Jakob): we're gonna have to make sure that courses don't have identical names, or even fileNames
	--> "a:sdf" vs "asdf" is okay, as "a_sdf.xml" vs "asdf.xml"
	--> "a:sdf" vs "a_sdf" is not okay, as "a_sdf.xml" vs "a_sdf.xml"
	--> create courseplay.courses.courseNameToCourse[courseName] (or [courseFileName]) -> when saving course in inputCourseNameDialogue (or even on every character change), check table, alert if duplicate name
function courseplay.courses:setCourseReferenceNodeIndex(course, index)
	assert(self.courseReferenceNodeIndexToCourse[index] == nil, ('ERROR: (course %q): another course with than nodeIndex (%d) already exists (%q)'):format(course.name, index, self.courseReferenceNodeIndexToCourse[index].name));

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


courseplay.courses:setup();