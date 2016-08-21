function split(pString, pPattern)
   local Table = {}  -- NOTE: use {n = 0} in Lua-5.0
   local fpat = "(.-)" .. pPattern
   local last_end = 1
   local s, e, cap = pString:find(fpat, 1)
   while s do
      if s ~= 1 or cap ~= "" then
     table.insert(Table,cap)
      end
      last_end = e+1
      s, e, cap = pString:find(fpat, last_end)
   end
   if last_end <= #pString then
      cap = pString:sub(last_end)
      table.insert(Table, cap)
   end
   return Table
end

f = assert (io.popen ("rosrun nn_pwc joy_publisher"))
  
local joy_vel_data = {}
 for line in f:lines() do
   print(line)
   joy_vel_data=split(line,' ')
   local joy_fb = tonumber(joy_vel_data[1])
   local joy_lr = tonumber(joy_vel_data[4])
   local vel_linear=tonumber(joy_vel_data[7])
   local vel_angular=tonumber(joy_vel_data[10])
   print(joy_fb)
   print(joy_lr)
   print(vel_linear)
   print(vel_angular)
end -- for loop
   
 f:close()


