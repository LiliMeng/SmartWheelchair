require 'gnuplot'
require 'rnn'
require 'nn'



--local dataLoader = require 'dataLoader'
--local ardUtils = require 'ardUtils'

local opt = lapp[[
-m, --modelLoc         (default "nil")    determines which dataset to use
-o, --order            (default "nil")    order of the inputs
--ard                                     use the differentials as input
]]

local ninputs=2
local noutputs=2
local frequency=16


local modelLoc = opt.modelLoc
local order = tonumber(opt.order)

if modelLoc == "nil" then
	error('Model location must be provided.')
end

if order == nil then
	error('Model order must be provided.')
end


save = torch.load(modelLoc)


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



function create_ar_sequence(inputs, targets, t, order)
  local dim_in = 2
  local dim_out = 2


  -- creates inputs[t-p:t-1] inclusive.
  -- pads the sequence with zeros if t-p < 1
  local chunk = inputs:narrow(1, math.max(1, t-order), math.min(t-1, order))
 
  local t_chunk = targets:narrow(1, math.max(1, t-order), math.min(t-1, order))

  return torch.cat(chunk:reshape(order * dim_in), t_chunk:reshape(order * dim_out), 1)
end
 




f = assert (io.popen ("rosrun nn_pwc joy_publisher"))

 inputs={}
 targets = {}
 predictions={}
local count=1
local joy_vel_data = {}
for line in f:lines() do
 --  print(line)
   local x = os.clock()
   joy_vel_data=split(line,' ')
   local joy_fb = tonumber(joy_vel_data[1])
   local joy_lr = tonumber(joy_vel_data[4])
   local vel_linear=tonumber(joy_vel_data[7])
   local vel_angular=tonumber(joy_vel_data[10])
   inputs[count] = {}
   targets[count] = {}
   predictions[count]={}
   local prediction = {}
   table.insert(inputs[count], vel_linear)
   table.insert(inputs[count], vel_angular)
   table.insert(targets[count], joy_fb)
   table.insert(targets[count], joy_lr)
   final_inputs=torch.Tensor(inputs)
   final_targets=torch.Tensor(targets)
   --print(final_inputs)
   --print(final_targets)
   
   
 --  print(targets)
  -- print(joy_fb)
  -- print(joy_lr)
  -- print(vel_linear)
  -- print(vel_angular)
    
 -- print(count)  
   count=count+1
   inputs[count] = nil
   
   if count>20 then
 
     dim_in = 2
     dim_out = 2
     model = save['model']

     local chunk = final_inputs:narrow(1, math.max(1, count-16), math.min(count-1, 16))
 
     local t_chunk = final_targets:narrow(1, math.max(1, count-16), math.min(count-1, 16))

     local ar_inputs = torch.cat(chunk:reshape(16 * dim_in), t_chunk:reshape(16 * dim_out), 1)
    
     prediction = model:forward(ar_inputs)
    
     print(prediction[1]*1000,",",prediction[2]*1000)

   end
   
 
 -- print(string.format("elapsed time: %.6f\n", os.clock() - x))


end -- for loop
   
f:close()

