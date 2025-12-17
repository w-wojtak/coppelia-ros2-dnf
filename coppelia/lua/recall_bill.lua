sim = require('sim')

function sysCall_init()
    -- 1. FORCE ARM TO REST (Statue Mode)
    local shoulder = sim.getObject('/Bill/rightShoulderJoint')
    local elbow = sim.getObject('/Bill/rightElbowJoint')
    
    -- Reset Shoulder to neutral (Arms at side)
    sim.setSphericalJointMatrix(shoulder, sim.buildMatrix({0,0,0}, {0,0,0}))
    
    -- Reset Elbow to straight
    sim.setJointPosition(elbow, 0)
    
    print('Bill is in Passive Mode.')


end