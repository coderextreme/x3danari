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

# --- Skeleton Definition ---
# The skeleton is defined first, establishing the hierarchy and joint centers.
# Per H-Anim spec, all 'center' values are relative to the HAnimHumanoid's coordinate system.

HAnimJoint5 = x3d.HAnimJoint()
HAnimJoint5.DEF = "hanim_sacroiliac"
HAnimJoint5.name = "sacroiliac"
HAnimJoint5.center = [0,0.9,0]

HAnimJoint6 = x3d.HAnimJoint()
HAnimJoint6.DEF = "hanim_vt1"
HAnimJoint6.name = "vt1"
HAnimJoint6.center = [0,1.15,0]
HAnimSegment7 = x3d.HAnimSegment(name="torso")
HAnimSite8 = x3d.HAnimSite()
HAnimSite8.translation = [0,1.15,0]
HAnimSite8.rotation = [1,0,0,3.14159]
Shape9 = x3d.Shape()
Appearance10 = x3d.Appearance()
Material11 = x3d.Material(diffuseColor = [0.6,0.6,0.6])
Appearance10.material = Material11
Shape9.appearance = Appearance10
Cone12 = x3d.Cone(height = 0.5, bottomRadius = 0.25)
Shape9.geometry = Cone12
HAnimSite8.children.append(Shape9)
HAnimSegment7.children.append(HAnimSite8)
HAnimJoint6.children.append(HAnimSegment7)

HAnimJoint13 = x3d.HAnimJoint()
HAnimJoint13.DEF = "hanim_skullbase"
HAnimJoint13.name = "skullbase"
HAnimJoint13.center = [0,1.5,0]
HAnimSegment14 = x3d.HAnimSegment(name="head")
HAnimSite15 = x3d.HAnimSite()
HAnimSite15.translation = [0,1.5,0]
Shape16 = x3d.Shape()
Appearance17 = x3d.Appearance()
Material18 = x3d.Material(diffuseColor = [0.6,0.6,0.6])
Appearance17.material = Material18
Shape16.appearance = Appearance17
Sphere19 = x3d.Sphere(radius = 0.15)
Shape16.geometry = Sphere19
HAnimSite15.children.append(Shape16)
HAnimSegment14.children.append(HAnimSite15)
HAnimJoint13.children.append(HAnimSegment14)
HAnimJoint6.children.append(HAnimJoint13)

HAnimJoint20 = x3d.HAnimJoint()
HAnimJoint20.DEF = "hanim_l_shoulder"
HAnimJoint20.name = "l_shoulder"
HAnimJoint20.center = [0.25,1.35,0]
HAnimSegment21 = x3d.HAnimSegment(name="l_upperarm")
HAnimSite22 = x3d.HAnimSite()
HAnimSite22.translation = [0.25,1.2,0]
Shape23 = x3d.Shape()
Appearance24 = x3d.Appearance()
Material25 = x3d.Material(diffuseColor = [0.6,0.6,0.6])
Appearance24.material = Material25
Shape23.appearance = Appearance24
Cylinder26 = x3d.Cylinder(height = 0.3, radius = 0.04)
Shape23.geometry = Cylinder26
HAnimSite22.children.append(Shape23)
HAnimSegment21.children.append(HAnimSite22)
HAnimJoint20.children.append(HAnimSegment21)

HAnimJoint27 = x3d.HAnimJoint()
HAnimJoint27.DEF = "hanim_l_elbow"
HAnimJoint27.name = "l_elbow"
HAnimJoint27.center = [0.25,1.05,0]
HAnimSegment28 = x3d.HAnimSegment(name="l_forearm")
HAnimSite29 = x3d.HAnimSite()
HAnimSite29.translation = [0.25,0.9,0]
Shape30 = x3d.Shape()
Appearance31 = x3d.Appearance()
Material32 = x3d.Material(diffuseColor = [0.6,0.6,0.6])
Appearance31.material = Material32
Shape30.appearance = Appearance31
Cylinder33 = x3d.Cylinder(height = 0.3, radius = 0.04)
Shape30.geometry = Cylinder33
HAnimSite29.children.append(Shape30)
HAnimSegment28.children.append(HAnimSite29)
HAnimJoint27.children.append(HAnimSegment28)
HAnimJoint34 = x3d.HAnimJoint(DEF = "hanim_l_wrist", name = "l_wrist", center = [0.25,0.75,0])
HAnimJoint27.children.append(HAnimJoint34)
HAnimJoint20.children.append(HAnimJoint27)
HAnimJoint6.children.append(HAnimJoint20)

HAnimJoint35 = x3d.HAnimJoint()
HAnimJoint35.DEF = "hanim_r_shoulder"
HAnimJoint35.name = "r_shoulder"
HAnimJoint35.center = [-0.25,1.35,0]
HAnimSegment36 = x3d.HAnimSegment(name="r_upperarm")
HAnimSite37 = x3d.HAnimSite()
HAnimSite37.translation = [-0.25,1.2,0]
Shape38 = x3d.Shape()
Appearance39 = x3d.Appearance()
Material40 = x3d.Material(diffuseColor = [0.6,0.6,0.6])
Appearance39.material = Material40
Shape38.appearance = Appearance39
Cylinder41 = x3d.Cylinder(height = 0.3, radius = 0.04)
Shape38.geometry = Cylinder41
HAnimSite37.children.append(Shape38)
HAnimSegment36.children.append(HAnimSite37)
HAnimJoint35.children.append(HAnimSegment36)

HAnimJoint42 = x3d.HAnimJoint()
HAnimJoint42.DEF = "hanim_r_elbow"
HAnimJoint42.name = "r_elbow"
HAnimJoint42.center = [-0.25,1.05,0]
HAnimSegment43 = x3d.HAnimSegment(name="r_forearm")
HAnimSite44 = x3d.HAnimSite()
HAnimSite44.translation = [-0.25,0.9,0]
Shape45 = x3d.Shape()
Appearance46 = x3d.Appearance()
Material47 = x3d.Material(diffuseColor = [0.6,0.6,0.6])
Appearance46.material = Material47
Shape45.appearance = Appearance46
Cylinder48 = x3d.Cylinder(height = 0.3, radius = 0.04)
Shape45.geometry = Cylinder48
HAnimSite44.children.append(Shape45)
HAnimSegment43.children.append(HAnimSite44)
HAnimJoint42.children.append(HAnimSegment43)
HAnimJoint49 = x3d.HAnimJoint(DEF = "hanim_r_wrist", name = "r_wrist", center = [-0.25,0.75,0])
HAnimJoint42.children.append(HAnimJoint49)
HAnimJoint35.children.append(HAnimJoint42)
HAnimJoint6.children.append(HAnimJoint35)
HAnimJoint5.children.append(HAnimJoint6)

HAnimJoint50 = x3d.HAnimJoint()
HAnimJoint50.DEF = "hanim_l_hip"
HAnimJoint50.name = "l_hip"
HAnimJoint50.center = [0.15,0.9,0]
HAnimSegment51 = x3d.HAnimSegment(name="l_thigh")
HAnimSite52 = x3d.HAnimSite()
HAnimSite52.translation = [0.15,0.7,0]
Shape53 = x3d.Shape()
Appearance54 = x3d.Appearance()
Material55 = x3d.Material(diffuseColor = [0.6,0.6,0.6])
Appearance54.material = Material55
Shape53.appearance = Appearance54
Cylinder56 = x3d.Cylinder(height = 0.4, radius = 0.05)
Shape53.geometry = Cylinder56
HAnimSite52.children.append(Shape53)
HAnimSegment51.children.append(HAnimSite52)
HAnimJoint50.children.append(HAnimSegment51)

HAnimJoint57 = x3d.HAnimJoint()
HAnimJoint57.DEF = "hanim_l_knee"
HAnimJoint57.name = "l_knee"
HAnimJoint57.center = [0.15,0.5,0]
HAnimSegment58 = x3d.HAnimSegment(name="l_calf")
HAnimSite59 = x3d.HAnimSite()
HAnimSite59.translation = [0.15,0.3,0]
Shape60 = x3d.Shape()
Appearance61 = x3d.Appearance()
Material62 = x3d.Material(diffuseColor = [0.6,0.6,0.6])
Appearance61.material = Material62
Shape60.appearance = Appearance61
Cylinder63 = x3d.Cylinder(height = 0.4, radius = 0.05)
Shape60.geometry = Cylinder63
HAnimSite59.children.append(Shape60)
HAnimSegment58.children.append(HAnimSite59)
HAnimJoint57.children.append(HAnimSegment58)
HAnimSegment64 = x3d.HAnimSegment(name="l_foot")
HAnimSite65 = x3d.HAnimSite()
HAnimSite65.translation = [0.15,0.1,0.05]
Shape66 = x3d.Shape()
Appearance67 = x3d.Appearance()
Material68 = x3d.Material(diffuseColor = [0.6,0.6,0.6])
Appearance67.material = Material68
Shape66.appearance = Appearance67
Sphere69 = x3d.Sphere(radius = 0.07)
Shape66.geometry = Sphere69
HAnimSite65.children.append(Shape66)
HAnimSegment64.children.append(HAnimSite65)
HAnimJoint57.children.append(HAnimSegment64)
HAnimJoint70 = x3d.HAnimJoint(DEF = "hanim_l_ankle", name = "l_ankle", center = [0.15,0.1,0])
HAnimJoint57.children.append(HAnimJoint70)
HAnimJoint50.children.append(HAnimJoint57)
HAnimJoint5.children.append(HAnimJoint50)

HAnimJoint71 = x3d.HAnimJoint()
HAnimJoint71.DEF = "hanim_r_hip"
HAnimJoint71.name = "r_hip"
HAnimJoint71.center = [-0.15,0.9,0]
HAnimSegment72 = x3d.HAnimSegment(name="r_thigh")
HAnimSite73 = x3d.HAnimSite()
HAnimSite73.translation = [-0.15,0.7,0]
Shape74 = x3d.Shape()
Appearance75 = x3d.Appearance()
Material76 = x3d.Material(diffuseColor = [0.6,0.6,0.6])
Appearance75.material = Material76
Shape74.appearance = Appearance75
Cylinder77 = x3d.Cylinder(height = 0.4, radius = 0.05)
Shape74.geometry = Cylinder77
HAnimSite73.children.append(Shape74)
HAnimSegment72.children.append(HAnimSite73)
HAnimJoint71.children.append(HAnimSegment72)

HAnimJoint78 = x3d.HAnimJoint()
HAnimJoint78.DEF = "hanim_r_knee"
HAnimJoint78.name = "r_knee"
HAnimJoint78.center = [-0.15,0.5,0]
HAnimSegment79 = x3d.HAnimSegment(name="r_calf")
HAnimSite80 = x3d.HAnimSite()
HAnimSite80.translation = [-0.15,0.3,0]
Shape81 = x3d.Shape()
Appearance82 = x3d.Appearance()
Material83 = x3d.Material(diffuseColor = [0.6,0.6,0.6])
Appearance82.material = Material83
Shape81.appearance = Appearance82
Cylinder84 = x3d.Cylinder(height = 0.4, radius = 0.05)
Shape81.geometry = Cylinder84
HAnimSite80.children.append(Shape81)
HAnimSegment79.children.append(HAnimSite80)
HAnimJoint78.children.append(HAnimSegment79)
HAnimSegment85 = x3d.HAnimSegment(name="r_foot")
HAnimSite86 = x3d.HAnimSite()
HAnimSite86.translation = [-0.15,0.1,0.05]
Shape87 = x3d.Shape()
Appearance88 = x3d.Appearance()
Material89 = x3d.Material(diffuseColor = [0.6,0.6,0.6])
Appearance88.material = Material89
Shape87.appearance = Appearance88
Sphere90 = x3d.Sphere(radius = 0.07)
Shape87.geometry = Sphere90
HAnimSite86.children.append(Shape87)
HAnimSegment85.children.append(HAnimSite86)
HAnimJoint78.children.append(HAnimSegment85)
HAnimJoint91 = x3d.HAnimJoint(DEF = "hanim_r_ankle", name = "r_ankle", center = [-0.15,0.1,0])
HAnimJoint78.children.append(HAnimJoint91)
HAnimJoint71.children.append(HAnimJoint78)
HAnimJoint5.children.append(HAnimJoint71)
HAnimHumanoid4.skeleton.append(HAnimJoint5)
Scene3.children.append(HAnimHumanoid4)

# --- Animation Timers, Interpolators, and Routes ---
TimeSensor107 = x3d.TimeSensor(DEF = "Clock", cycleInterval = 2, loop = True)
Scene3.children.append(TimeSensor107)
OrientationInterpolator108 = x3d.OrientationInterpolator(DEF = "LHipAnimator", key = [0,0.25,0.5,0.75,1], keyValue = [(1, 0, 0, -0.5),(1, 0, 0, 0),(1, 0, 0, 0.5),(1, 0, 0, 0),(1, 0, 0, -0.5)])
Scene3.children.append(OrientationInterpolator108)
OrientationInterpolator109 = x3d.OrientationInterpolator(DEF = "RHipAnimator", key = [0,0.25,0.5,0.75,1], keyValue = [(1, 0, 0, 0.5),(1, 0, 0, 0),(1, 0, 0, -0.5),(1, 0, 0, 0),(1, 0, 0, 0.5)])
Scene3.children.append(OrientationInterpolator109)
OrientationInterpolator110 = x3d.OrientationInterpolator(DEF = "KneeAnimator", key = [0,0.25,0.5,0.75,1], keyValue = [(1, 0, 0, 0),(1, 0, 0, 1.2),(1, 0, 0, 0),(1, 0, 0, 0),(1, 0, 0, 0)])
Scene3.children.append(OrientationInterpolator110)
OrientationInterpolator111 = x3d.OrientationInterpolator(DEF = "LShoulderAnimator", key = [0,0.25,0.5,0.75,1], keyValue = [(1, 0, 0, 0.4),(1, 0, 0, 0),(1, 0, 0, -0.4),(1, 0, 0, 0),(1, 0, 0, 0.4)])
Scene3.children.append(OrientationInterpolator111)
OrientationInterpolator112 = x3d.OrientationInterpolator(DEF = "RShoulderAnimator", key = [0,0.25,0.5,0.75,1], keyValue = [(1, 0, 0, -0.4),(1, 0, 0, 0),(1, 0, 0, 0.4),(1, 0, 0, 0),(1, 0, 0, -0.4)])
Scene3.children.append(OrientationInterpolator112)
OrientationInterpolator113 = x3d.OrientationInterpolator(DEF = "ElbowAnimator", key = [0,0.25,0.5,0.75,1], keyValue = [(1, 0, 0, -0.3),(1, 0, 0, -0.8),(1, 0, 0, -0.3),(1, 0, 0, -0.3),(1, 0, 0, -0.3)])
Scene3.children.append(OrientationInterpolator113)
ROUTE114 = x3d.ROUTE(fromNode = "Clock", fromField = "fraction_changed", toNode = "LHipAnimator", toField = "set_fraction")
Scene3.children.append(ROUTE114)
ROUTE115 = x3d.ROUTE(fromNode = "LHipAnimator", fromField = "value_changed", toNode = "hanim_l_hip", toField = "set_rotation")
Scene3.children.append(ROUTE115)
ROUTE116 = x3d.ROUTE(fromNode = "Clock", fromField = "fraction_changed", toNode = "RHipAnimator", toField = "set_fraction")
Scene3.children.append(ROUTE116)
ROUTE117 = x3d.ROUTE(fromNode = "RHipAnimator", fromField = "value_changed", toNode = "hanim_r_hip", toField = "set_rotation")
Scene3.children.append(ROUTE117)
ROUTE118 = x3d.ROUTE(fromNode = "Clock", fromField = "fraction_changed", toNode = "KneeAnimator", toField = "set_fraction")
Scene3.children.append(ROUTE118)
ROUTE119 = x3d.ROUTE(fromNode = "KneeAnimator", fromField = "value_changed", toNode = "hanim_l_knee", toField = "set_rotation")
Scene3.children.append(ROUTE119)
ROUTE120 = x3d.ROUTE(fromNode = "KneeAnimator", fromField = "value_changed", toNode = "hanim_r_knee", toField = "set_rotation")
Scene3.children.append(ROUTE120)
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
f = open("walking_man_gemini.new.python.x3d", mode="w", encoding="utf-8")
f.write(X3D0.XML())
f.close()
