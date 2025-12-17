sim = require('sim')

-- === CONFIGURATION ===
local jobQueue = {'/Cuboid1', '/Cuboid2', '/Cuboid3'}
local defaultMoveDist = 0.35 -- Standard distance for most cubes

-- CALIBRATION
local shoulderY = -0.17  
local anglePerMeter = 1.6 

-- === FINE TUNING ===
local cubeTweaks = {
    ['/Cuboid1'] = {
        pitchOffset = -0.0, 
        sideOffset = 0.2,   -- Your fix for the reach
        moveDist = 0.55     -- <--- NEW: Move Cuboid 1 much further (55cm)
    },
    ['/Cuboid2'] = {
        pitchOffset = 0.0,   
        sideOffset = 0.0, 
        moveDist = 0.35     -- Standard distance
    },
    ['/Cuboid3'] = {
        pitchOffset = 0.0,   
        sideOffset = 0.0,
        moveDist = 0.35     -- Standard distance
    }
}

-- POSES
local poses = {
    hoverPitch = -1.15, 
    hoverElbow = -0.05,
    grabPitch  = -0.90,
    grabElbow  = -0.02,
    liftPitch  = -0.50,
    liftElbow  = -1.60,
    placePitch = -1.10
}

local shoulder, elbow, hand

function sysCall_init()
    shoulder = sim.getObject('/Bill/rightShoulderJoint')
    elbow = sim.getObject('/Bill/rightElbowJoint')
    hand = sim.getObject('/Bill/rightHand_tip')
    print('=== BILL V3: CUSTOM PLACEMENT ===')
end

function moveArm(sideAngle, forwardAngle, elbowAngle, duration)
    local st = sim.getSimulationTime()
    while (sim.getSimulationTime() - st) < duration do
        sim.step()
    end
    local rotMatrix = sim.buildMatrix({0,0,0}, {sideAngle, forwardAngle, 0})
    sim.setSphericalJointMatrix(shoulder, rotMatrix)
    sim.setJointPosition(elbow, elbowAngle)
end

function getSideAngleForY(targetY)
    local dy = targetY - shoulderY
    return -0.1 + (dy * anglePerMeter)
end

function sysCall_thread()
    print('Waiting 2 seconds...')
    moveArm(0, 0, 0, 0.1)
    sim.wait(2)

    for i, cubeName in ipairs(jobQueue) do
        print('----------------------------------')
        print('TARGETING: ' .. cubeName)
        
        local status, cubeHandle = pcall(sim.getObject, cubeName)
        
        if status then
            local startPos = sim.getObjectPosition(cubeHandle, -1)
            local targetZ = startPos[3] 
            
            -- LOAD TWEAKS
            local tweaks = cubeTweaks[cubeName] or {pitchOffset=0, sideOffset=0, moveDist=defaultMoveDist}
            
            -- Use specific move distance for this cube, or default
            local thisMoveDist = tweaks.moveDist or defaultMoveDist

            -- 1. CALCULATE ANGLES
            local pickSideAngle = getSideAngleForY(startPos[2])
            -- Calculate place angle using the CUSTOM distance
            local placeSideAngle = getSideAngleForY(startPos[2] + thisMoveDist)
            
            -- Apply Offsets
            local finalHoverPitch = poses.hoverPitch + tweaks.pitchOffset
            local finalGrabPitch  = poses.grabPitch + tweaks.pitchOffset
            local finalSideAngle  = pickSideAngle + tweaks.sideOffset

            -- STAGE 1: HOVER
            print('1. Hovering...')
            moveArm(finalSideAngle, finalHoverPitch, poses.hoverElbow, 1.5)
            sim.wait(0.5) 
            
            -- STAGE 2: DESCEND
            print('2. Descending...')
            moveArm(finalSideAngle, finalGrabPitch, poses.grabElbow, 0.5)
            sim.wait(0.5)

            -- GRASP CHECK
            local hPos = sim.getObjectPosition(hand, -1)
            local cPos = sim.getObjectPosition(cubeHandle, -1)
            local dist = math.sqrt((hPos[1]-cPos[1])^2 + (hPos[2]-cPos[2])^2 + (hPos[3]-cPos[3])^2)
            
            if dist < 0.45 then 
                sim.setObjectParent(cubeHandle, hand, true)
                print('3. GRASPED!')
                
                -- LIFT
                print('4. Lifting...')
                moveArm(finalSideAngle, poses.liftPitch, poses.liftElbow, 1.0)
                sim.wait(0.5)
                
                -- MOVE SIDEWAYS (To the new custom location)
                print('5. Moving Sideways (' .. thisMoveDist .. 'm)...')
                moveArm(placeSideAngle, poses.liftPitch, poses.liftElbow, 1.5)
                sim.wait(0.5)
                
                -- PLACE
                print('6. Placing...')
                moveArm(placeSideAngle, poses.placePitch, poses.grabElbow, 1.0)
                sim.wait(0.8) 
                
                -- SNAP & RELEASE
                local currentRot = sim.getObjectOrientation(cubeHandle, -1)
                local currentPos = sim.getObjectPosition(cubeHandle, -1)
                sim.setObjectOrientation(cubeHandle, -1, {0, 0, currentRot[3]})
                sim.setObjectPosition(cubeHandle, -1, {currentPos[1], currentPos[2], targetZ})
                
                sim.setObjectParent(cubeHandle, -1, true)
                print('7. Released.')
            else
                print('FAIL: Too far ('..dist..').')
            end
            
            -- RETRACT
            print('8. Retracting...')
            moveArm(0, 0, 0, 1.5)
            sim.wait(1.0)
        else
            print('ERROR: ' .. cubeName .. ' not found.')
        end
    end
    print('=== ALL TASKS COMPLETED ===')
end