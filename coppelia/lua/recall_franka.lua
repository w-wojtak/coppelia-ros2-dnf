sim = require('sim')

local jointHandles = {}
local maxVel, maxAccel, maxJerk = {}, {}, {}

-- === CONFIGURATIONS FROM OPTIMIZER ===

-- Grab configurations (at table)
local configsGrab = {
    Cuboid1 = {0.35, 0.65, 0.00, -2.05, 0.00, 2.50, 0.00},
    Cuboid2 = {0.50, 0.70, 0.00, -2.00, 0.00, 2.70, 0.00},
    Cuboid3 = {0.65, 0.80, 0.00, -1.85, 0.00, 2.80, 0.00}
}

-- Pick configurations (above table - reduced J2 by 0.1)
local configsAbove = {
    Cuboid1 = {0.35, 0.55, 0.00, -2.05, 0.00, 2.50, 0.00},
    Cuboid2 = {0.50, 0.60, 0.00, -2.00, 0.00, 2.70, 0.00},
    Cuboid3 = {0.65, 0.70, 0.00, -1.85, 0.00, 2.80, 0.00}
}

-- Destination place configurations (at table)
local configsDestPlace = {
    Cuboid1 = {-0.20, 0.645, 0.00, -2.10, 0.00, 2.55, 0.00},
    Cuboid2 = {0.00, 0.585, 0.00, -2.20, 0.00, 2.60, 0.00},
    Cuboid3 = {0.20, 0.57, 0.00, -2.20, 0.00, 2.65, 0.00}
}

-- Destination above configurations (above table - reduced J2 by 0.1)
local configsDestAbove = {
    Cuboid1 = {-0.20, 0.55, 0.00, -2.10, 0.00, 2.55, 0.00},
    Cuboid2 = {0.00, 0.55, 0.00, -2.20, 0.00, 2.60, 0.00},
    Cuboid3 = {0.20, 0.55, 0.00, -2.20, 0.00, 2.65, 0.00}
}

local configHome = {0, 0, 0, -1.57, 0, 1.57, 0}

-- State
local state = 'idle'
local corout = nil
local attachedCube = nil

function sysCall_init()
    for i = 1, 7 do
        jointHandles[i] = sim.getObject('../joint', {index = i-1})
    end
    
    local vel = 200
    local accel = 100
    local jerk = 200
    for i = 1, 7 do
        maxVel[i] = vel * math.pi / 180
        maxAccel[i] = accel * math.pi / 180
        maxJerk[i] = jerk * math.pi / 180
    end
    
    print('Franka controller initialized with optimized configs')
end

function moveRobot(targetConfig)
    local params = {
        joints = jointHandles,
        targetPos = targetConfig,
        maxVel = maxVel,
        maxAccel = maxAccel,
        maxJerk = maxJerk
    }
    sim.moveToConfig(params)
end

function attachCube(cubeName)
    local cubeHandle = sim.getObject('/' .. cubeName)
    local connHandle = sim.getObject('/Franka/connection')
    
    -- Preserve global position when parenting
    sim.setObjectParent(cubeHandle, connHandle, true)
    attachedCube = cubeHandle
    print('Attached: ' .. cubeName)
end

function detachCube()
    if attachedCube then
        -- Preserve global position when unparenting
        sim.setObjectParent(attachedCube, -1, true)
        print('Detached cube')
        attachedCube = nil
    end
end

function sysCall_actuation()
    if state == 'idle' then
        local cmd = sim.getStringSignal('frankaCommand')
        
        if cmd and cmd ~= '' then
            sim.clearStringSignal('frankaCommand')
            
            if configsGrab[cmd] then
                print('=== Picking up: ' .. cmd .. ' ===')
                state = 'busy'
                
                corout = coroutine.create(function()
                    -- 1. Move above pick location
                    moveRobot(configsAbove[cmd])
                    
                    -- 2. Lower to grab
                    moveRobot(configsGrab[cmd])
                    
                    -- 3. Attach cube
                    attachCube(cmd)
                    
                    -- 4. Lift up
                    moveRobot(configsAbove[cmd])
                    
                    -- 5. Move above destination
                    moveRobot(configsDestAbove[cmd])
                    
                    -- 6. Lower to place
                    moveRobot(configsDestPlace[cmd])
                    
                    -- 7. Detach cube
                    detachCube()
                    
                    -- 8. Lift up
                    moveRobot(configsDestAbove[cmd])
                    
                    -- 9. Return home
                    moveRobot(configHome)
                    
                    print('=== Completed: ' .. cmd .. ' ===')
                    state = 'idle'
                end)
            else
                print('Unknown command: ' .. cmd)
            end
        end
    end
    
    if corout and coroutine.status(corout) ~= 'dead' then
        local ok, err = coroutine.resume(corout)
        if not ok then
            print('Error: ' .. tostring(err))
            detachCube()
            state = 'idle'
            corout = nil
        end
    end
end

function sysCall_cleanup()
    detachCube()
    print('Franka controller stopped')
end