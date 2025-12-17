sim = require('sim')

-- "Safe Pose" Angles (Retracted high and back)
local safePose = {0, -0.6, 0, -2.2, 0, 1.8, 0.78}
local joints = {}

function sysCall_init()
    -- 1. Get the handle of the object this script is attached to (The Franka Base)
    local baseHandle = sim.getObject('.')
    
    -- 2. Find ALL joints inside this model hierarchy automatically
    -- This works regardless of what the joints are named (joint, Franka_joint1, etc.)
    local allJoints = sim.getObjectsInTree(baseHandle, sim.object_joint_type)
    
    -- 3. We only need the first 7 joints (The Arm)
    if #allJoints >= 7 then
        for i = 1, 7 do
            joints[i] = allJoints[i]
        end
        print('SUCCESS: Found Franka joints. Entering Safe Mode.')
    else
        print('ERROR: Could not find 7 joints in the model hierarchy!')
    end
end

function sysCall_actuation()
    -- 4. Actively hold the safe position
    if #joints >= 7 then
        for i = 1, 7 do
            sim.setJointTargetPosition(joints[i], safePose[i])
        end
    end
end