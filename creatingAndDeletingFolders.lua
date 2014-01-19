--[[
	local dir = g_careerScreen.savegames[g_careerScreen.selectedIndex].savegameDirectory .. '/testFolder';
	local dir2 = dir .. '/anotherFolder';
	local testFilePath = dir .. '/folderExists.xml';
	local testFileCopyPath = dir2 .. '/folderExistsCopy.xml';

	createFolder(dir);
	print('createFolder("' .. dir .. '")');
	createFolder(dir2);
	print('createFolder("' .. dir2 .. '")');

	-- local testFile = io.open(testFilePath, 'w');
	-- print(string.format('fileExists(%q) [after io.open()] = %s', testFilePath, tostring(fileExists(testFilePath))));

	local testFile = createXMLFile('CPTEST', testFilePath, 'CP');
	print(string.format('testFile=%s, fileExists(%q) [after createXMLFile()] = %s', tostring(testFile), testFilePath, tostring(fileExists(testFilePath))));

	setXMLInt(testFile, 'CP.courses.course#parent', 456);

	saveXMLFile(testFile);
	print(string.format('testFile=%s, fileExists(%q) [after saveXMLFile()] = %s', tostring(testFile), testFilePath, tostring(fileExists(testFilePath))));

	-- testFile:close();
	-- print(string.format('fileExists(%q) [after io:close()] = %s', testFilePath, tostring(fileExists(testFilePath))));

	copyFile(testFilePath, testFileCopyPath, true);
	print(string.format('fileExists(%q) [after copyFile(..., false)] = %s', testFilePath, tostring(fileExists(testFilePath))));

	delete(testFile);
	print(string.format('fileExists(%q) [after delete()] = %s', testFilePath, tostring(fileExists(testFilePath))));

	-- deleteFile(testFilePath);
	-- print(string.format('fileExists(%q) [after deleteFile()] = %s', testFilePath, tostring(fileExists(testFilePath))));

	-- deleteFolder(dir);
	-- print(string.format('fileExists(%q) [after deleteFolder()] = %s', testFilePath, tostring(fileExists(testFilePath))));
--]]


--[[
* createXMLFile(name, filePath, baseStr) --e.g. createXMLFile('testFile', 'testFile.xml', 'ASDF') -- returns fileId
* deleteFile(filePath) --only works with createXMLFile --removes the file LITERALLY
* delete(fileId) --removes the file VIRTUALLY
* copyFile(origFilePath [str], newFilePath [str], overwrite [bool])
* how to move a single file?
	copyFile -> deleteFile


INTERESTING:
createXMLFile
loadXMLFile
loadXMLFileFromMemory
saveXMLFile
removeXMLProperty
getFileMD5
getFiles
loadfile
setOverlayRotation
]]

g_cp = {};
local g_cp_mt = Class(g_cp);
addModEventListener(g_cp);

function g_cp:addFolderToHierarchy(folderTable, id)
	if not self.addedFoldersById[id] then
		-- print(('-'):rep(50) .. ' addFolderToHierarchy(' .. id .. ') start')
		local folder = folderTable[id];
		local parentId = folder.parent;
		-- print(string.format('id=%d, parentId=%d', id, parentId));
		if not self.addedFoldersById[parentId] then
			-- print('\tadd parent first')
			self:addFolderToHierarchy(folderTable, parentId);
		end;

		local parentAddressStr = self.folderAddressStrById[parentId];
		-- print(string.format('\tparentAddressStr=%q', parentAddressStr));
		local a = table.map(Utils.splitString('>', parentAddressStr), tonumber);
		local numLevels = #a;

		local parentAddress = self.folderHierarchy;

		-- would be possible with a for loop, but too much conflict potential
		if numLevels == 1 then
			parentAddress = self.folderHierarchy[ a[1] ];
		elseif numLevels == 2 then
			parentAddress = self.folderHierarchy[ a[1] ].children[ a[2] ];
		elseif numLevels == 3 then
			parentAddress = self.folderHierarchy[ a[1] ].children[ a[2] ].children[ a[3] ];
		elseif numLevels == 4 then
			parentAddress = self.folderHierarchy[ a[1] ].children[ a[2] ].children[ a[3] ].children[ a[4] ];
		elseif numLevels == 5 then
			parentAddress = self.folderHierarchy[ a[1] ].children[ a[2] ].children[ a[3] ].children[ a[4] ].children[ a[5] ];
		elseif numLevels == 6 then
			parentAddress = self.folderHierarchy[ a[1] ].children[ a[2] ].children[ a[3] ].children[ a[4] ].children[ a[5] ].children[ a[6] ];
		elseif numLevels == 7 then
			parentAddress = self.folderHierarchy[ a[1] ].children[ a[2] ].children[ a[3] ].children[ a[4] ].children[ a[5] ].children[ a[6] ].children[ a[7] ];
		elseif numLevels == 8 then
			parentAddress = self.folderHierarchy[ a[1] ].children[ a[2] ].children[ a[3] ].children[ a[4] ].children[ a[5] ].children[ a[6] ].children[ a[7] ].children[ a[8] ];
		elseif numLevels == 9 then
			parentAddress = self.folderHierarchy[ a[1] ].children[ a[2] ].children[ a[3] ].children[ a[4] ].children[ a[5] ].children[ a[6] ].children[ a[7] ].children[ a[8] ];
		elseif numLevels == 10 then
			parentAddress = self.folderHierarchy[ a[1] ].children[ a[2] ].children[ a[3] ].children[ a[4] ].children[ a[5] ].children[ a[6] ].children[ a[7] ].children[ a[8] ].children[ a[9] ];
		else
			--TOO DEEP, PULL OUT!
		end;

		local address = parentAddress.children;
		-- print(tableShow(address, 'address after loop'))
		-- print(tableShow(parentAddress, 'parentAddress after loop'))

		address[id] = {
			id = id;
			name = folder.name;
			parentId = parentId;
			parentName = parentAddress.name;
			path = parentAddress.path .. '/' .. folder.name;
			parentPath = parentAddress.path;
			parentAddress = parentAddress;
			children = {};
		};
		-- print(tableShow(address[id], 'address[id] after setting empty table at id index'))

		self.addedFoldersById[id] = true;
		self.folderAddressStrById[id] = self.folderAddressStrById[parentId] .. '>' .. id;
		self.folderIdToAddress[id] = address[id];
		self.folderIdToPath[id] = address[id].path;
		self.folderIdToName[id] = folder.name;

		-- print(('-'):rep(50) .. ' addFolderToHierarchy(' .. id .. ') done')
	end;
end;

function g_cp:convertFoldersFlatToHierarchical(folderTable)
	self.addedFoldersById = {
		[0] = true
	};
	self.folderAddressStrById = {
		[0] = '0';
	};

	self.folderHierarchy[0] = {
		id = 0;
		name = 'topLevel';
		path = self.topLevelPath;
		children = {};
	};
	self.folderIdToAddress[0] = self.folderHierarchy[0];
	self.folderIdToPath[0] = self.folderHierarchy[0].path;
	self.folderIdToName[0] = self.folderHierarchy[0].name;


	for id,data in pairs(folderTable) do
		self:addFolderToHierarchy(folderTable, id);
	end;
end;

function g_cp:createFolder(folderId)
	print(string.format('g_cp:createFolder(%d) -> path=%q', folderId, self.folderIdToPath[folderId]));
	createFolder(self.folderIdToPath[folderId]);
	print(string.format('\tcreate "folderExists.xml" file -> path=%q', self.folderIdToPath[folderId] .. '/folderExists.xml'));
	self.folderIdToAddress[folderId].folderExistsFile = createXMLFile('cpFolder_' .. folderId .. '_exists', self.folderIdToPath[folderId] .. '/folderExists.xml', 'cpFolder_' .. folderId .. '_exists')
	saveXMLFile(self.folderIdToAddress[folderId].folderExistsFile);
end;

function g_cp:removeFolder(folderId)
	deleteFolder(self.folderIdToPath[folderId]);
end;

function g_cp:createCourseFile(courseData)
	--[[
	1. get parent's path
	2. createXMLFile('cpCourse_' .. courseId, path, 'course')
	]]
end;

function g_cp:moveCourseFile(courseData, newParentId)
	--[[
	1. get old path
	2. get new path (via newParentId's path)
	3. copyFile(oldFilePath, newFilePath, true)
	4. deleteFile(oldFilePath)
	]]
end;

function g_cp:deleteCourseFile(courseData)
	--[[
	1. get file path
	(2. delete(fileId))
	3. deleteFile(filePath)
	]]
end;

function g_cp:initialSetup()
	self.topLevelPath = g_careerScreen.savegames[g_careerScreen.selectedIndex].savegameDirectory .. '/courseplay';
	print(string.format('g_cp.topLevelPath=%q', tostring(self.topLevelPath)));

	self.folderHierarchy = {};
	self.folderIdToAddress = {};
	self.folderIdToPath = {};
	self.folderIdToName = {};

	local foldersFlat = {
		[1] = { 
			id = 1,
			parent = 2,
			name = 'folder01'
		};
		[2] = { 
			id = 2,
			parent = 0,
			name = 'folder02'
		};
		[3] = { 
			id = 3,
			parent = 1,
			name = 'folder03'
		};
		[4] = { 
			id = 4,
			parent = 3,
			name = 'folder04'
		};
		[5] = { 
			id = 6,
			parent = 3,
			name = 'folder05'
		};
		[6] = { 
			id = 6,
			parent = 0,
			name = 'folder06'
		};
		[7] = { 
			id = 7,
			parent = 1,
			name = 'folder07'
		};
		[8] = { 
			id = 8,
			parent = 6,
			name = 'folder08'
		};
	};

	self:convertFoldersFlatToHierarchical(foldersFlat); --creates self.folderHierarchy, self.folderIdToAddress, self.folderIdToPath, self.folderIdToName
	-- print(tableShow(self.folderHierarchy, 'self.folderHierarchy'));
	-- print(tableShow(self.folderIdToAddress, 'self.folderIdToAddress'));
	print(tableShow(self.folderIdToPath, 'self.folderIdToPath'));
	-- print(tableShow(self.folderIdToName, 'self.folderIdToName'));

	-- self:createFolder(0);
	for id,childFolders in pairs(self.folderHierarchy) do
		self:createInitialFolders(id);
	end;
end;

function g_cp:createInitialFolders(folderId)
	-- print(string.format('createInitialFolders(%d)', folderId));
	self:createFolder(folderId);
	local folder = self.folderIdToAddress[folderId];
	-- print(tableShow(folder, 'folder ' .. folderId, nil, nil, 2));
	for id,childFolder in pairs(folder.children) do
		-- print(string.format('call createInitialFolders(%d)', id));
		self:createInitialFolders(id);
	end;
end;

function g_cp:draw()
end;

function g_cp:mouseEvent(posX, posY, isDown, isUp, button)
end;

function g_cp:update(dt)
end;

function g_cp:updateTick(dt)
end;

function g_cp:keyEvent()
end;

