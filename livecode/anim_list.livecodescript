script "anim_list"
-- anim_list: a game's Animation List (TODO A-84, Opus 15 Nov 2026). The Vibecode3D Inspector's Animation page
-- writes LIB_GAMES/Vibecode3D_Games/Animations/<Game>.anims.txt, one line an animation:
--   name TAB LIB_MOCAP clip TAB in TAB out TAB loop        ('#' lines are comments)
-- so a model INHERITS its moves from the game's list (owner: "instead of embedded directly in them"):
-- the clip stays in LIB_MOCAP/library, the list keeps only which frames to use.
--   start using stack ".../LIB_MOCAP/livecode/anim_list.livecodescript"
--   put Anims.Read(tListFile) into tA            -- tA[name][clip|in|out|loop]
--   put Anims.Frames(tProjects, tA["jump"]) into tF   -- tF[1..n] = "x,y,z,x,y,z,..." lines; tF["fps"], tF["edges"]...

function Anims.Read pFile
   local tA, tLine
   if there is not a file pFile then return empty
   set the itemDelimiter to tab
   repeat for each line tLine in textDecode(url ("binfile:" & pFile), "UTF-8")
      if tLine is empty or char 1 of tLine is "#" then next repeat
      put item 2 of tLine into tA[item 1 of tLine]["clip"]
      put item 3 of tLine into tA[item 1 of tLine]["in"]
      put item 4 of tLine into tA[item 1 of tLine]["out"]
      put (item 5 of tLine is true) into tA[item 1 of tLine]["loop"]
   end repeat
   return tA
end Anims.Read

-- the frames from In to Out of one list entry, renumbered from 1, with the clip's header values
function Anims.Frames pProjects, pEntry
   local tFile, tLine, n, k, tOut, tKey
   put pProjects & "/LIB_MOCAP/library/" & pEntry["clip"] & "/" & pEntry["clip"] & "_stick.txt" into tFile
   if there is not a file tFile then return empty
   put 0 into n
   put 0 into k
   repeat for each line tLine in textDecode(url ("binfile:" & tFile), "UTF-8")
      if tLine is empty or char 1 of tLine is "#" then next repeat
      put item 1 of tLine into tKey
      if tKey is a number then
         add 1 to n
         if n >= pEntry["in"] and n <= pEntry["out"] then
            add 1 to k
            put tLine into tOut[k]
         end if
      else
         put item 2 to -1 of tLine into tOut[tKey]
      end if
   end repeat
   put k into tOut["frames"]
   put pEntry["loop"] into tOut["loop"]
   return tOut
end Anims.Frames
