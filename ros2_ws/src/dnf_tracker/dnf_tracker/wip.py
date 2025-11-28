sim=require'sim'
simIK=require'simIK'

-- === 1. SUPER SEARCH FUNCTION ===
-- This ignores paths (like ./ or /Bill/) and finds the object by name anywhere
function findGlobal(name)
    local i = 0
    while true do
        local h = sim.getObjects(i, sim.handle_all)
        if h == -1 then break end
        local alias = sim.getObjectAlias(h, 0) -- 0 means just the name (e.g. "hands_demo")
        
        -- Check if name matches exactly
        if alias == name then
            print("SUCCESS: Found '" .. name .. "'")
            return h
        end
        i = i + 1
    end
    print("FAILED: Could not find any object named '" .. name .. "'")
    return -1
end

-- === 2. MOVEMENT & GRASPING ===
function movCallback(data)
    if hands ~= -1 then
        sim.setObjectPose(hands, data.pose, modelBase)
        simIK.handleGroup(ikEnv, ikGroup, {syncWorlds = true, allowError = true})
    end
    
    -- FAKE MAGNET GRASPING
    if objectBeingPushed and objectBeingPushed ~= -1 and rightHandTip ~= -1 then
        local handPos = sim.getObjectPosition(rightHandTip, -1)
        local cubePos = sim.getObjectPosition(objectBeingPushed, -1)
        -- Snap cube to hand X/Y, keep Cube Z height
        sim.setObjectPosition(objectBeingPushed, -1, {handPos[1], handPos[2] + 0.05, cubePos[3]})
    end
end

function moveHands(targetPosition, speed)
    if hands == -1 then return end -- Safety check
    
    local currentPose=sim.getObjectPose(hands,modelBase)
    local targetPose=table.clone(currentPose)
    targetPose[1]=targetPosition[1]
    targetPose[2]=targetPosition[2]
    targetPose[3]=targetPosition[3]
    
    local params = {
        pose = currentPose,
        targetPose = targetPose,
        maxVel = {speed or 0.4},
        maxAccel = {0.5},
        maxJerk = {0.1},
        callback = movCallback
    }
    sim.moveToPose(params)
end

function sysCall_thread()
    print("=== SCRIPT STARTING ===")
    modelBase = sim.getObject('.')
    
    -- === 3. ROBUST SETUP ===
    -- We use findGlobal to hunt down the objects
    
    hands = findGlobal('hands_demo')
    rightHandTip = findGlobal('rightHand_tip')
    
    -- Safety Stop
    if hands == -1 or rightHandTip == -1 then
        print("!!! CRITICAL ERROR !!!")
        print("The script cannot find 'hands_demo' or 'rightHand_tip'.")
        print("Please check the Scene Hierarchy and make sure those names are spelled EXACTLY like that.")
        return
    end

    -- Find other IK parts (We check if they exist before adding them)
    local simLeftHandTip = findGlobal('leftHand_tip')
    local simLeftHandTarget = findGlobal('leftHand_target')
    local simRightHandTarget = findGlobal('rightHand_target')
    local simPostureTip = findGlobal('posture_tip')
    local simPostureTarget = findGlobal('posture_target')
    
    -- Body parts
    local simLeftUpperArm = findGlobal('leftUpperArm')
    local simLeftArmBendingTip = findGlobal('leftArmBending_tip')
    local simLeftArmBendingTarget = findGlobal('leftArmBending_target')
    local simRightUpperArm = findGlobal('rightUpperArm')
    local simRightArmBendingTip = findGlobal('rightArmBending_tip')
    local simRightArmBendingTarget = findGlobal('rightArmBending_target')
    local simLowerLegs = findGlobal('lowerLegs')
    local simKneeBendingTip = findGlobal('kneeBending_tip')
    local simKneeBendingTarget = findGlobal('kneeBending_target')
    local simBillBody = findGlobal('body')
    
    -- Elbows (Try both spellings)
    local simLeftElbowTip = findGlobal('leftEllbowPreferredOrientation_tip')
    if simLeftElbowTip == -1 then simLeftElbowTip = findGlobal('leftElbowPreferredOrientation_tip') end
    local simLeftElbowTarget = findGlobal('leftEllbowPreferredOrientation_target')
    if simLeftElbowTarget == -1 then simLeftElbowTarget = findGlobal('leftElbowPreferredOrientation_target') end
    
    local simRightElbowTip = findGlobal('rightEllbowPreferredOrientation_tip')
    if simRightElbowTip == -1 then simRightElbowTip = findGlobal('rightElbowPreferredOrientation_tip') end
    local simRightElbowTarget = findGlobal('rightEllbowPreferredOrientation_target')
    if simRightElbowTarget == -1 then simRightElbowTarget = findGlobal('rightElbowPreferredOrientation_target') end

    -- === 4. IK INITIALIZATION ===
    ikEnv=simIK.createEnvironment()
    ikGroup=simIK.createGroup(ikEnv)
    simIK.setGroupCalculation(ikEnv,ikGroup,simIK.method_damped_least_squares,0.1,10)
    
    -- ONLY add constraints for objects that were actually found
    if simLeftHandTip~=-1 and simLeftHandTarget~=-1 then simIK.addElementFromScene(ikEnv,ikGroup,modelBase,simLeftHandTip,simLeftHandTarget,simIK.constraint_position) end
    if rightHandTip~=-1 and simRightHandTarget~=-1 then simIK.addElementFromScene(ikEnv,ikGroup,modelBase,rightHandTip,simRightHandTarget,simIK.constraint_position) end
    if simPostureTip~=-1 and simPostureTarget~=-1 then simIK.addElementFromScene(ikEnv,ikGroup,modelBase,simPostureTip,simPostureTarget,simIK.constraint_x+simIK.constraint_alpha_beta) end
    
    if simLeftUpperArm~=-1 and simLeftArmBendingTip~=-1 then simIK.addElementFromScene(ikEnv,ikGroup,simLeftUpperArm,simLeftArmBendingTip,simLeftArmBendingTarget,simIK.constraint_gamma) end
    if simRightUpperArm~=-1 and simRightArmBendingTip~=-1 then simIK.addElementFromScene(ikEnv,ikGroup,simRightUpperArm,simRightArmBendingTip,simRightArmBendingTarget,simIK.constraint_gamma) end
    if simLowerLegs~=-1 and simKneeBendingTip~=-1 then simIK.addElementFromScene(ikEnv,ikGroup,simLowerLegs,simKneeBendingTip,simKneeBendingTarget,simIK.constraint_gamma) end
    
    if simBillBody~=-1 and simLeftElbowTip~=-1 then simIK.addElementFromScene(ikEnv,ikGroup,simBillBody,simLeftElbowTip,simLeftElbowTarget,simIK.constraint_orientation) end
    if simBillBody~=-1 and simRightElbowTip~=-1 then simIK.addElementFromScene(ikEnv,ikGroup,simBillBody,simRightElbowTip,simRightElbowTarget,simIK.constraint_orientation) end

    -- === 5. FIND DUMMIES ===
    local d1 = findGlobal('dummy1')
    local d2 = findGlobal('dummy2')
    local d3 = findGlobal('dummy3')
    
    objectBeingPushed = nil 
    
    -- === 6. EXECUTE MOVES ===
    print("Bill is ready. Moving to start position...")
    moveHands({0.3, 0, 1.1}, 0.5) 
    sim.wait(1)

    local function performPush(dummyObj)
        if dummyObj == -1 then return end
        local pos = sim.getObjectPosition(dummyObj, -1)
        
        -- 1. Approach (Behind and Left of cube)
        moveHands({pos[1] - 0.25, pos[2] - 0.2, 1.0}, 0.5) 
        
        -- 2. Push
        objectBeingPushed = dummyObj
        moveHands({pos[1] - 0.25, pos[2] + 0.3, 1.0}, 0.2)
        objectBeingPushed = nil
        
        -- 3. Lift
        moveHands({pos[1] - 0.25, pos[2] + 0.3, 1.15}, 0.5)
    end

    if d1 ~= -1 then print("Pushing Cube 1"); performPush(d1) end
    if d2 ~= -1 then print("Pushing Cube 2"); performPush(d2) end
    if d3 ~= -1 then print("Pushing Cube 3"); performPush(d3) end
    
    print("Sequence Finished.")
    moveHands({0.3, 0, 1.1}, 0.5)
end

function sysCall_cleanup()
end