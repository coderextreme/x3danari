print('<!--')
import x3d
print('-->')
X3D0 = x3d.X3D()
X3D0.profile = "Immersive"
X3D0.version = "4.0"
head1 = x3d.head()
component2 = x3d.component()
component2.name = "HAnim"
component2.level = 1

head1.children.append(component2)

X3D0.head = head1
Scene3 = x3d.Scene()
HAnimHumanoid4 = x3d.HAnimHumanoid()
HAnimHumanoid4.DEF = "HAnim"
HAnimHumanoid4.name = "humanoid"

# --- Skeleton Definition (Unchanged) ---
HAnimJoint5 = x3d.HAnimJoint(DEF = "hanim_sacroiliac", name = "sacroiliac", center = [0,0.9,0])
HAnimJoint6 = x3d.HAnimJoint(DEF = "hanim_vt1", name = "vt1", center = [0,1.15,0])
HAnimSegment7 = x3d.HAnimSegment(name="torso")
HAnimSite8 = x3d.HAnimSite(translation = [0,1.15,0], rotation = [1,0,0,3.14159])
Shape9 = x3d.Shape()
Appearance10 = x3d.Appearance(material = x3d.Material(diffuseColor = [0.6,0.6,0.6]))
Shape9.appearance = Appearance10
Cone12 = x3d.Cone(height = 0.5, bottomRadius = 0.25)
Shape9.geometry = Cone12
HAnimSite8.children.append(Shape9)
HAnimSegment7.children.append(HAnimSite8)
HAnimJoint6.children.append(HAnimSegment7)
HAnimJoint13 = x3d.HAnimJoint(DEF = "hanim_skullbase", name = "skullbase", center = [0,1.5,0])
HAnimSegment14 = x3d.HAnimSegment(name="head")
HAnimSite15 = x3d.HAnimSite(translation = [0,1.5,0])
Shape16 = x3d.Shape()
Appearance17 = x3d.Appearance(material = x3d.Material(diffuseColor = [0.6,0.6,0.6]))
Shape16.appearance = Appearance17
Sphere19 = x3d.Sphere(radius = 0.15)
Shape16.geometry = Sphere19
HAnimSite15.children.append(Shape16)
HAnimSegment14.children.append(HAnimSite15)
HAnimJoint13.children.append(HAnimSegment14)
HAnimJoint6.children.append(HAnimJoint13)
HAnimJoint20 = x3d.HAnimJoint(DEF = "hanim_l_shoulder", name = "l_shoulder", center = [0.25,1.35,0])
HAnimSegment21 = x3d.HAnimSegment(name="l_upperarm")
HAnimSite22 = x3d.HAnimSite(translation = [0.25,1.2,0])
Shape23 = x3d.Shape(appearance = x3d.Appearance(material = x3d.Material(diffuseColor = [0.6,0.6,0.6])), geometry = x3d.Cylinder(height = 0.3, radius = 0.04))
HAnimSite22.children.append(Shape23)
HAnimSegment21.children.append(HAnimSite22)
HAnimJoint20.children.append(HAnimSegment21)
HAnimJoint27 = x3d.HAnimJoint(DEF = "hanim_l_elbow", name = "l_elbow", center = [0.25,1.05,0])
HAnimSegment28 = x3d.HAnimSegment(name="l_forearm")
HAnimSite29 = x3d.HAnimSite(translation = [0.25,0.9,0])
Shape30 = x3d.Shape(appearance = x3d.Appearance(material = x3d.Material(diffuseColor = [0.6,0.6,0.6])), geometry = x3d.Cylinder(height = 0.3, radius = 0.04))
HAnimSite29.children.append(Shape30)
HAnimSegment28.children.append(HAnimSite29)
HAnimJoint27.children.append(HAnimSegment28)
HAnimJoint34 = x3d.HAnimJoint(DEF = "hanim_l_wrist", name = "l_wrist", center = [0.25,0.75,0])
HAnimJoint27.children.append(HAnimJoint34)
HAnimJoint20.children.append(HAnimJoint27)
HAnimJoint6.children.append(HAnimJoint20)
HAnimJoint35 = x3d.HAnimJoint(DEF = "hanim_r_shoulder", name = "r_shoulder", center = [-0.25,1.35,0])
HAnimSegment36 = x3d.HAnimSegment(name="r_upperarm")
HAnimSite37 = x3d.HAnimSite(translation = [-0.25,1.2,0])
Shape38 = x3d.Shape(appearance = x3d.Appearance(material = x3d.Material(diffuseColor = [0.6,0.6,0.6])), geometry = x3d.Cylinder(height = 0.3, radius = 0.04))
HAnimSite37.children.append(Shape38)
HAnimSegment36.children.append(HAnimSite37)
HAnimJoint35.children.append(HAnimSegment36)
HAnimJoint42 = x3d.HAnimJoint(DEF = "hanim_r_elbow", name = "r_elbow", center = [-0.25,1.05,0])
HAnimSegment43 = x3d.HAnimSegment(name="r_forearm")
HAnimSite44 = x3d.HAnimSite(translation = [-0.25,0.9,0])
Shape45 = x3d.Shape(appearance = x3d.Appearance(material = x3d.Material(diffuseColor = [0.6,0.6,0.6])), geometry = x3d.Cylinder(height = 0.3, radius = 0.04))
HAnimSite44.children.append(Shape45)
HAnimSegment43.children.append(HAnimSite44)
HAnimJoint42.children.append(HAnimSegment43)
HAnimJoint49 = x3d.HAnimJoint(DEF = "hanim_r_wrist", name = "r_wrist", center = [-0.25,0.75,0])
HAnimJoint42.children.append(HAnimJoint49)
HAnimJoint35.children.append(HAnimJoint42)
HAnimJoint6.children.append(HAnimJoint35)
HAnimJoint5.children.append(HAnimJoint6)
HAnimJoint50 = x3d.HAnimJoint(DEF = "hanim_l_hip", name = "l_hip", center = [0.15,0.9,0])
HAnimSegment51 = x3d.HAnimSegment(name="l_thigh")
HAnimSite52 = x3d.HAnimSite(translation = [0.15,0.7,0])
Shape53 = x3d.Shape(appearance = x3d.Appearance(material = x3d.Material(diffuseColor = [0.6,0.6,0.6])), geometry = x3d.Cylinder(height = 0.4, radius = 0.05))
HAnimSite52.children.append(Shape53)
HAnimSegment51.children.append(HAnimSite52)
HAnimJoint50.children.append(HAnimSegment51)
HAnimJoint57 = x3d.HAnimJoint(DEF = "hanim_l_knee", name = "l_knee", center = [0.15,0.5,0])
HAnimSegment58 = x3d.HAnimSegment(name="l_calf")
HAnimSite59 = x3d.HAnimSite(translation = [0.15,0.3,0])
Shape60 = x3d.Shape(appearance = x3d.Appearance(material = x3d.Material(diffuseColor = [0.6,0.6,0.6])), geometry = x3d.Cylinder(height = 0.4, radius = 0.05))
HAnimSite59.children.append(Shape60)
HAnimSegment58.children.append(HAnimSite59)
HAnimJoint57.children.append(HAnimSegment58)
HAnimJoint70 = x3d.HAnimJoint(DEF = "hanim_l_ankle", name = "l_ankle", center = [0.15,0.1,0])
HAnimSegment64 = x3d.HAnimSegment(name="l_foot")
HAnimSite65 = x3d.HAnimSite(translation = [0.15,0.1,0.05])
Shape66 = x3d.Shape(appearance = x3d.Appearance(material = x3d.Material(diffuseColor = [0.6,0.6,0.6])), geometry = x3d.Sphere(radius = 0.07))
HAnimSite65.children.append(Shape66)
HAnimSegment64.children.append(HAnimSite65)
HAnimJoint70.children.append(HAnimSegment64) # foot attached to ankle
HAnimJoint57.children.append(HAnimJoint70)
HAnimJoint50.children.append(HAnimJoint57)
HAnimJoint5.children.append(HAnimJoint50)
HAnimJoint71 = x3d.HAnimJoint(DEF = "hanim_r_hip", name = "r_hip", center = [-0.15,0.9,0])
HAnimSegment72 = x3d.HAnimSegment(name="r_thigh")
HAnimSite73 = x3d.HAnimSite(translation = [-0.15,0.7,0])
Shape74 = x3d.Shape(appearance = x3d.Appearance(material = x3d.Material(diffuseColor = [0.6,0.6,0.6])), geometry = x3d.Cylinder(height = 0.4, radius = 0.05))
HAnimSite73.children.append(Shape74)
HAnimSegment72.children.append(HAnimSite73)
HAnimJoint71.children.append(HAnimSegment72)
HAnimJoint78 = x3d.HAnimJoint(DEF = "hanim_r_knee", name = "r_knee", center = [-0.15,0.5,0])
HAnimSegment79 = x3d.HAnimSegment(name="r_calf")
HAnimSite80 = x3d.HAnimSite(translation = [-0.15,0.3,0])
Shape81 = x3d.Shape(appearance = x3d.Appearance(material = x3d.Material(diffuseColor = [0.6,0.6,0.6])), geometry = x3d.Cylinder(height = 0.4, radius = 0.05))
HAnimSite80.children.append(Shape81)
HAnimSegment79.children.append(HAnimSite80)
HAnimJoint78.children.append(HAnimSegment79)
HAnimJoint91 = x3d.HAnimJoint(DEF = "hanim_r_ankle", name = "r_ankle", center = [-0.15,0.1,0])
HAnimSegment85 = x3d.HAnimSegment(name="r_foot")
HAnimSite86 = x3d.HAnimSite(translation = [-0.15,0.1,0.05])
Shape87 = x3d.Shape(appearance = x3d.Appearance(material = x3d.Material(diffuseColor = [0.6,0.6,0.6])), geometry = x3d.Sphere(radius = 0.07))
HAnimSite86.children.append(Shape87)
HAnimSegment85.children.append(HAnimSite86)
HAnimJoint91.children.append(HAnimSegment85) # foot attached to ankle
HAnimJoint78.children.append(HAnimJoint91)
HAnimJoint71.children.append(HAnimJoint78)
HAnimJoint5.children.append(HAnimJoint71)
HAnimHumanoid4.skeleton.append(HAnimJoint5)
Scene3.children.append(HAnimHumanoid4)

# --- Animation Timers, Interpolators, and Routes (MODIFIED SECTION) ---
TimeSensor107 = x3d.TimeSensor(DEF = "Clock", cycleInterval = 2, loop = True)
Scene3.children.append(TimeSensor107)

# Animators for hips and shoulders (Unchanged)
OrientationInterpolator108 = x3d.OrientationInterpolator(DEF = "LHipAnimator", key = [0,0.25,0.5,0.75,1], keyValue = [(1, 0, 0, -0.5),(1, 0, 0, 0),(1, 0, 0, 0.5),(1, 0, 0, 0),(1, 0, 0, -0.5)])
Scene3.children.append(OrientationInterpolator108)
OrientationInterpolator109 = x3d.OrientationInterpolator(DEF = "RHipAnimator", key = [0,0.25,0.5,0.75,1], keyValue = [(1, 0, 0, 0.5),(1, 0, 0, 0),(1, 0, 0, -0.5),(1, 0, 0, 0),(1, 0, 0, 0.5)])
Scene3.children.append(OrientationInterpolator109)
OrientationInterpolator111 = x3d.OrientationInterpolator(DEF = "LShoulderAnimator", key = [0,0.25,0.5,0.75,1], keyValue = [(1, 0, 0, 0.4),(1, 0, 0, 0),(1, 0, 0, -0.4),(1, 0, 0, 0),(1, 0, 0, 0.4)])
Scene3.children.append(OrientationInterpolator111)
OrientationInterpolator112 = x3d.OrientationInterpolator(DEF = "RShoulderAnimator", key = [0,0.25,0.5,0.75,1], keyValue = [(1, 0, 0, -0.4),(1, 0, 0, 0),(1, 0, 0, 0.4),(1, 0, 0, 0),(1, 0, 0, -0.4)])
Scene3.children.append(OrientationInterpolator112)
OrientationInterpolator113 = x3d.OrientationInterpolator(DEF = "ElbowAnimator", key = [0,0.25,0.5,0.75,1], keyValue = [(1, 0, 0, -0.3),(1, 0, 0, -0.8),(1, 0, 0, -0.3),(1, 0, 0, -0.3),(1, 0, 0, -0.3)])
Scene3.children.append(OrientationInterpolator113)

# --- START OF FIX ---
# We need two separate, phase-shifted knee animators.
# The right leg is forward at t=0, so the right knee should be bent.
# The left leg is forward at t=0.5, so the left knee should be bent.

# Left Knee Animator: Bends when left leg is forward (around t=0.5)
LKneeAnimator = x3d.OrientationInterpolator(DEF = "LKneeAnimator", 
    key = [0, 0.25, 0.5, 0.75, 1], 
    keyValue = [(1, 0, 0, 0), (1, 0, 0, 0), (1, 0, 0, 1.2), (1, 0, 0, 0), (1, 0, 0, 0)])
Scene3.children.append(LKneeAnimator)

# Right Knee Animator: Bends when right leg is forward (around t=0 and t=1)
RKneeAnimator = x3d.OrientationInterpolator(DEF = "RKneeAnimator", 
    key = [0, 0.25, 0.5, 0.75, 1], 
    keyValue = [(1, 0, 0, 1.2), (1, 0, 0, 0), (1, 0, 0, 0), (1, 0, 0, 0), (1, 0, 0, 1.2)])
Scene3.children.append(RKneeAnimator)
# --- END OF FIX ---


# --- Routes (MODIFIED SECTION) ---
ROUTE114 = x3d.ROUTE(fromNode = "Clock", fromField = "fraction_changed", toNode = "LHipAnimator", toField = "set_fraction")
Scene3.children.append(ROUTE114)
ROUTE115 = x3d.ROUTE(fromNode = "LHipAnimator", fromField = "value_changed", toNode = "hanim_l_hip", toField = "set_rotation")
Scene3.children.append(ROUTE115)
ROUTE116 = x3d.ROUTE(fromNode = "Clock", fromField = "fraction_changed", toNode = "RHipAnimator", toField = "set_fraction")
Scene3.children.append(ROUTE116)
ROUTE117 = x3d.ROUTE(fromNode = "RHipAnimator", fromField = "value_changed", toNode = "hanim_r_hip", toField = "set_rotation")
Scene3.children.append(ROUTE117)

# --- START OF FIX ---
# Route to the new, separate knee animators
ROUTE118a = x3d.ROUTE(fromNode = "Clock", fromField = "fraction_changed", toNode = "LKneeAnimator", toField = "set_fraction")
Scene3.children.append(ROUTE118a)
ROUTE119 = x3d.ROUTE(fromNode = "LKneeAnimator", fromField = "value_changed", toNode = "hanim_l_knee", toField = "set_rotation")
Scene3.children.append(ROUTE119)
ROUTE118b = x3d.ROUTE(fromNode = "Clock", fromField = "fraction_changed", toNode = "RKneeAnimator", toField = "set_fraction")
Scene3.children.append(ROUTE118b)
ROUTE120 = x3d.ROUTE(fromNode = "RKneeAnimator", fromField = "value_changed", toNode = "hanim_r_knee", toField = "set_rotation")
Scene3.children.append(ROUTE120)
# --- END OF FIX ---

ROUTE121 = x3d.ROUTE(fromNode = "Clock", fromField = "fraction_changed", toNode = "LShoulderAnimator", toField = "set_fraction")
Scene3.children.append(ROUTE121)
ROUTE122 = x3d.ROUTE(fromNode = "LShoulderAnimator", fromField = "value_changed", toNode = "hanim_l_shoulder", toField = "set_rotation")
Scene3.children.append(ROUTE122)
ROUTE123 = x3d.ROUTE(fromNode = "Clock", fromField = "fraction_changed", toNode = "RShoulderAnimator", toField = "set_fraction")
Scene3.children.append(ROUTE123)
ROUTE124 = x3d.ROUTE(fromNode = "RShoulderAnimator", fromField = "value_changed", toNode = "hanim_r_shoulder", toField = "set_rotation")
Scene3.children.append(ROUTE124)
ROUTE125 = x3d.ROUTE(fromNode = "Clock", fromField = "fraction_changed", toNode = "ElbowAnimator", toField = "set_fraction")
Scene3.children.append(ROUTE125)
ROUTE126 = x3d.ROUTE(fromNode = "ElbowAnimator", fromField = "value_changed", toNode = "hanim_l_elbow", toField = "set_rotation")
Scene3.children.append(ROUTE126)
ROUTE127 = x3d.ROUTE(fromNode = "ElbowAnimator", fromField = "value_changed", toNode = "hanim_r_elbow", toField = "set_rotation")
Scene3.children.append(ROUTE127)

X3D0.Scene = Scene3
f = open("walking_man_gemini3.new.python.x3d", mode="w", encoding="utf-8")
f.write(X3D0.XML())
f.close()
