--[[ LIB_MOCAP: ask iGame3D itself where the bones of a WIF animation are.

Run:  igame3dvs.exe "<path>\ig3d_bones.lua"     (from the iGame3D 2022 base folder)
Writes C:/LIB_MCP/ig3d_bones.txt, then quits. tools/wif_check.py compares those world
positions with tools/wif.py's BVH of the same animation, to prove the conversion against
the engine that made the tables (not only against the 16 surviving original BVHs).

Lines written:  anim,<model>,<animation>   then per frame and bone
                time,<seconds since the animation started>,<bone>,<parent>,<x>,<y>,<z>
Physics are not used (they are broken on current Windows).
--]]

local OUT = "C:/LIB_MCP/ig3d_bones.txt"
local TESTS = {
	{ model = "WIF/Walk_Slow", anim = "drunk1BVH3560", keys = { 0, 5, 10, 20, 40, 60, 80 } },
	{ model = "WIF/Walk_Backward", anim = "MIKE8BVH1440", keys = { 0, 5, 10, 15, 20, 30 } },
	{ model = "WIF/Fight_Bugs", anim = "beesBVH8360", keys = { 0, 10, 50, 100, 150, 200 } },
}

gObjects = {}
ig3d_SetAutoBoneUpdating_b(true)
local f = io.open(OUT, "w")
if f == nil then quit() end
f:write("# iGame3D bone world positions (getBoneInfo IG3D_POSITION)\n")

-- iGame3D plays an animation on its own clock from the moment setObjectInfo starts it
-- (ig3d_UpdateObjectBonesAtTime_si does not move that clock here), so each animation is
-- played for SECONDS of real time and every frame is logged with its elapsed time.
local SECONDS = 4.0
local test, t0, elapsed = 0, nil, 0
local cur = {}

local function bonesOf(name)
	local bones = {}
	for line in string.gmatch(ig3d_GetObjectBonesList_s_s(name) or "", "[^\r\n]+") do
		local bn, par = string.match(line, "^(%S+)%s+(%S+)")
		bones[#bones + 1] = { name = bn or line, parent = par or "" }
	end
	return bones
end

local function nextTest()
	test = test + 1
	if test > #TESTS then
		f:write("done\n")
		f:close()
		quit()
		return false
	end
	local t = TESTS[test]
	if cur.obj then setObjectInfo(cur.obj, IG3D_POSITION, 1000, 0, 0) end   -- out of the way
	cur.obj = make(ig3d_object, t.model)
	cur.name = getObjectInfo(cur.obj, IG3D_NAME)
	cur.bones = bonesOf(cur.name)
	f:write("anim," .. t.model .. "," .. t.anim .. "," .. #cur.bones .. " bones\n")
	setObjectInfo(cur.obj, IG3D_ANIMATION, t.anim, 0, false)
	elapsed = 0
	passed()
	return true
end

function lib_mocap_loop()
	if test == 0 then
		if not nextTest() then return end
		return
	end
	elapsed = elapsed + passed()
	for i, b in ipairs(cur.bones) do
		local bone = get(ig3d_bone, cur.obj, i - 1)
		local x, y, z = getBoneInfo(bone, IG3D_POSITION)
		f:write(string.format("time,%.4f,%s,%s,%.4f,%.4f,%.4f\n", elapsed, b.name, b.parent, x or 0, y or 0, z or 0))
	end
	if elapsed > SECONDS then nextTest() end
end

ig3d_SetMode_i(2)
edit_func = lib_mocap_loop
game_func = lib_mocap_loop
