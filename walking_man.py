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
HAnimJoint5 = x3d.HAnimJoint()
HAnimJoint5.DEF = "hanim_sacroiliac"
HAnimJoint5.name = "sacroiliac"
HAnimJoint5.center = [0,0.9,0]
HAnimJoint6 = x3d.HAnimJoint()
HAnimJoint6.DEF = "hanim_vt1"
HAnimJoint6.name = "vt1"
HAnimJoint6.center = [0,1.15,0]
HAnimSegment7 = x3d.HAnimSegment()
HAnimSite8 = x3d.HAnimSite()
HAnimSite8.translation = [0,1.15,0]
HAnimSite8.rotation = [1,0,0,3.14159]
Shape9 = x3d.Shape()
Appearance10 = x3d.Appearance()
Material11 = x3d.Material()
Material11.diffuseColor = [0.6,0.6,0.6]

Appearance10.material = Material11

Shape9.appearance = Appearance10
Cone12 = x3d.Cone()
Cone12.height = 0.5
Cone12.bottomRadius = 0.25

Shape9.geometry = Cone12

HAnimSite8.children.append(Shape9)

HAnimSegment7.children.append(HAnimSite8)

HAnimJoint6.children.append(HAnimSegment7)
HAnimJoint13 = x3d.HAnimJoint()
HAnimJoint13.DEF = "hanim_skullbase"
HAnimJoint13.name = "skullbase"
HAnimJoint13.center = [0,1.5,0]
HAnimSegment14 = x3d.HAnimSegment()
HAnimSite15 = x3d.HAnimSite()
HAnimSite15.translation = [0,1.5,0]
Shape16 = x3d.Shape()
Appearance17 = x3d.Appearance()
Material18 = x3d.Material()
Material18.diffuseColor = [0.6,0.6,0.6]

Appearance17.material = Material18

Shape16.appearance = Appearance17
Sphere19 = x3d.Sphere()
Sphere19.radius = 0.15

Shape16.geometry = Sphere19

HAnimSite15.children.append(Shape16)

HAnimSegment14.children.append(HAnimSite15)

HAnimJoint13.children.append(HAnimSegment14)

HAnimJoint6.children.append(HAnimJoint13)
HAnimJoint20 = x3d.HAnimJoint()
HAnimJoint20.DEF = "hanim_l_shoulder"
HAnimJoint20.name = "l_shoulder"
HAnimJoint20.rotation = [1,0,0,5.98318530717959]
HAnimJoint20.center = [0.25,1.35,0]
HAnimSegment21 = x3d.HAnimSegment()
HAnimSite22 = x3d.HAnimSite()
HAnimSite22.translation = [0.25,1.2,0]
Shape23 = x3d.Shape()
Appearance24 = x3d.Appearance()
Material25 = x3d.Material()
Material25.diffuseColor = [0.6,0.6,0.6]

Appearance24.material = Material25

Shape23.appearance = Appearance24
Cylinder26 = x3d.Cylinder()
Cylinder26.height = 0.3
Cylinder26.radius = 0.04

Shape23.geometry = Cylinder26

HAnimSite22.children.append(Shape23)

HAnimSegment21.children.append(HAnimSite22)

HAnimJoint20.children.append(HAnimSegment21)
HAnimJoint27 = x3d.HAnimJoint()
HAnimJoint27.DEF = "hanim_l_elbow"
HAnimJoint27.name = "l_elbow"
HAnimJoint27.rotation = [0.999999999999997,0,0,5.98318530717959]
HAnimJoint27.center = [0.25,1.05,0]
HAnimSegment28 = x3d.HAnimSegment()
HAnimSite29 = x3d.HAnimSite()
HAnimSite29.translation = [0.25,0.9,0]
Shape30 = x3d.Shape()
Appearance31 = x3d.Appearance()
Material32 = x3d.Material()
Material32.diffuseColor = [0.6,0.6,0.6]

Appearance31.material = Material32

Shape30.appearance = Appearance31
Cylinder33 = x3d.Cylinder()
Cylinder33.height = 0.3
Cylinder33.radius = 0.04

Shape30.geometry = Cylinder33

HAnimSite29.children.append(Shape30)

HAnimSegment28.children.append(HAnimSite29)

HAnimJoint27.children.append(HAnimSegment28)
HAnimJoint34 = x3d.HAnimJoint()
HAnimJoint34.DEF = "hanim_l_wrist"
HAnimJoint34.name = "l_wrist"
HAnimJoint34.center = [0.25,0.75,0]

HAnimJoint27.children.append(HAnimJoint34)

HAnimJoint20.children.append(HAnimJoint27)

HAnimJoint6.children.append(HAnimJoint20)
HAnimJoint35 = x3d.HAnimJoint()
HAnimJoint35.DEF = "hanim_r_shoulder"
HAnimJoint35.name = "r_shoulder"
HAnimJoint35.rotation = [1,0,0,0.299999999999999]
HAnimJoint35.center = [-0.25,1.35,0]
HAnimSegment36 = x3d.HAnimSegment()
HAnimSite37 = x3d.HAnimSite()
HAnimSite37.translation = [-0.25,1.2,0]
Shape38 = x3d.Shape()
Appearance39 = x3d.Appearance()
Material40 = x3d.Material()
Material40.diffuseColor = [0.6,0.6,0.6]

Appearance39.material = Material40

Shape38.appearance = Appearance39
Cylinder41 = x3d.Cylinder()
Cylinder41.height = 0.3
Cylinder41.radius = 0.04

Shape38.geometry = Cylinder41

HAnimSite37.children.append(Shape38)

HAnimSegment36.children.append(HAnimSite37)

HAnimJoint35.children.append(HAnimSegment36)
HAnimJoint42 = x3d.HAnimJoint()
HAnimJoint42.DEF = "hanim_r_elbow"
HAnimJoint42.name = "r_elbow"
HAnimJoint42.rotation = [0.999999999999997,0,0,5.98318530717959]
HAnimJoint42.center = [-0.25,1.05,0]
HAnimSegment43 = x3d.HAnimSegment()
HAnimSite44 = x3d.HAnimSite()
HAnimSite44.translation = [-0.25,0.9,0]
Shape45 = x3d.Shape()
Appearance46 = x3d.Appearance()
Material47 = x3d.Material()
Material47.diffuseColor = [0.6,0.6,0.6]

Appearance46.material = Material47

Shape45.appearance = Appearance46
Cylinder48 = x3d.Cylinder()
Cylinder48.height = 0.3
Cylinder48.radius = 0.04

Shape45.geometry = Cylinder48

HAnimSite44.children.append(Shape45)

HAnimSegment43.children.append(HAnimSite44)

HAnimJoint42.children.append(HAnimSegment43)
HAnimJoint49 = x3d.HAnimJoint()
HAnimJoint49.DEF = "hanim_r_wrist"
HAnimJoint49.name = "r_wrist"
HAnimJoint49.center = [-0.25,0.75,0]

HAnimJoint42.children.append(HAnimJoint49)

HAnimJoint35.children.append(HAnimJoint42)

HAnimJoint6.children.append(HAnimJoint35)

HAnimJoint5.children.append(HAnimJoint6)
HAnimJoint50 = x3d.HAnimJoint()
HAnimJoint50.DEF = "hanim_l_hip"
HAnimJoint50.name = "l_hip"
HAnimJoint50.rotation = [1,0,0,0.375]
HAnimJoint50.center = [0.15,0.9,0]
HAnimSegment51 = x3d.HAnimSegment()
HAnimSite52 = x3d.HAnimSite()
HAnimSite52.translation = [0.15,0.7,0]
Shape53 = x3d.Shape()
Appearance54 = x3d.Appearance()
Material55 = x3d.Material()
Material55.diffuseColor = [0.6,0.6,0.6]

Appearance54.material = Material55

Shape53.appearance = Appearance54
Cylinder56 = x3d.Cylinder()
Cylinder56.height = 0.4
Cylinder56.radius = 0.05

Shape53.geometry = Cylinder56

HAnimSite52.children.append(Shape53)

HAnimSegment51.children.append(HAnimSite52)

HAnimJoint50.children.append(HAnimSegment51)
HAnimJoint57 = x3d.HAnimJoint()
HAnimJoint57.DEF = "hanim_l_knee"
HAnimJoint57.name = "l_knee"
HAnimJoint57.center = [0.15,0.5,0]
HAnimSegment58 = x3d.HAnimSegment()
HAnimSite59 = x3d.HAnimSite()
HAnimSite59.translation = [0.15,0.3,0]
Shape60 = x3d.Shape()
Appearance61 = x3d.Appearance()
Material62 = x3d.Material()
Material62.diffuseColor = [0.6,0.6,0.6]

Appearance61.material = Material62

Shape60.appearance = Appearance61
Cylinder63 = x3d.Cylinder()
Cylinder63.height = 0.4
Cylinder63.radius = 0.05

Shape60.geometry = Cylinder63

HAnimSite59.children.append(Shape60)

HAnimSegment58.children.append(HAnimSite59)

HAnimJoint57.children.append(HAnimSegment58)
HAnimSegment64 = x3d.HAnimSegment()
HAnimSite65 = x3d.HAnimSite()
HAnimSite65.translation = [0.15,0.1,0.05]
Shape66 = x3d.Shape()
Appearance67 = x3d.Appearance()
Material68 = x3d.Material()
Material68.diffuseColor = [0.6,0.6,0.6]

Appearance67.material = Material68

Shape66.appearance = Appearance67
Sphere69 = x3d.Sphere()
Sphere69.radius = 0.07

Shape66.geometry = Sphere69

HAnimSite65.children.append(Shape66)

HAnimSegment64.children.append(HAnimSite65)

HAnimJoint57.children.append(HAnimSegment64)
HAnimJoint70 = x3d.HAnimJoint()
HAnimJoint70.DEF = "hanim_l_ankle"
HAnimJoint70.name = "l_ankle"
HAnimJoint70.center = [0.15,0.1,0]

HAnimJoint57.children.append(HAnimJoint70)

HAnimJoint50.children.append(HAnimJoint57)

HAnimJoint5.children.append(HAnimJoint50)
HAnimJoint71 = x3d.HAnimJoint()
HAnimJoint71.DEF = "hanim_r_hip"
HAnimJoint71.name = "r_hip"
HAnimJoint71.rotation = [1,0,0,5.90818530717959]
HAnimJoint71.center = [-0.15,0.9,0]
HAnimSegment72 = x3d.HAnimSegment()
HAnimSite73 = x3d.HAnimSite()
HAnimSite73.translation = [-0.15,0.7,0]
Shape74 = x3d.Shape()
Appearance75 = x3d.Appearance()
Material76 = x3d.Material()
Material76.diffuseColor = [0.6,0.6,0.6]

Appearance75.material = Material76

Shape74.appearance = Appearance75
Cylinder77 = x3d.Cylinder()
Cylinder77.height = 0.4
Cylinder77.radius = 0.05

Shape74.geometry = Cylinder77

HAnimSite73.children.append(Shape74)

HAnimSegment72.children.append(HAnimSite73)

HAnimJoint71.children.append(HAnimSegment72)
HAnimJoint78 = x3d.HAnimJoint()
HAnimJoint78.DEF = "hanim_r_knee"
HAnimJoint78.name = "r_knee"
HAnimJoint78.center = [-0.15,0.5,0]
HAnimSegment79 = x3d.HAnimSegment()
HAnimSite80 = x3d.HAnimSite()
HAnimSite80.translation = [-0.15,0.3,0]
Shape81 = x3d.Shape()
Appearance82 = x3d.Appearance()
Material83 = x3d.Material()
Material83.diffuseColor = [0.6,0.6,0.6]

Appearance82.material = Material83

Shape81.appearance = Appearance82
Cylinder84 = x3d.Cylinder()
Cylinder84.height = 0.4
Cylinder84.radius = 0.05

Shape81.geometry = Cylinder84

HAnimSite80.children.append(Shape81)

HAnimSegment79.children.append(HAnimSite80)

HAnimJoint78.children.append(HAnimSegment79)
HAnimSegment85 = x3d.HAnimSegment()
HAnimSite86 = x3d.HAnimSite()
HAnimSite86.translation = [-0.15,0.1,0.05]
Shape87 = x3d.Shape()
Appearance88 = x3d.Appearance()
Material89 = x3d.Material()
Material89.diffuseColor = [0.6,0.6,0.6]

Appearance88.material = Material89

Shape87.appearance = Appearance88
Sphere90 = x3d.Sphere()
Sphere90.radius = 0.07

Shape87.geometry = Sphere90

HAnimSite86.children.append(Shape87)

HAnimSegment85.children.append(HAnimSite86)

HAnimJoint78.children.append(HAnimSegment85)
HAnimJoint91 = x3d.HAnimJoint()
HAnimJoint91.DEF = "hanim_r_ankle"
HAnimJoint91.name = "r_ankle"
HAnimJoint91.center = [-0.15,0.1,0]

HAnimJoint78.children.append(HAnimJoint91)

HAnimJoint71.children.append(HAnimJoint78)

HAnimJoint5.children.append(HAnimJoint71)

HAnimHumanoid4.skeleton.append(HAnimJoint5)
HAnimJoint92 = x3d.HAnimJoint()
HAnimJoint92.USE = "hanim_sacroiliac"

HAnimHumanoid4.joints.append(HAnimJoint92)
HAnimJoint93 = x3d.HAnimJoint()
HAnimJoint93.USE = "hanim_vt1"

HAnimHumanoid4.joints.append(HAnimJoint93)
HAnimJoint94 = x3d.HAnimJoint()
HAnimJoint94.USE = "hanim_skullbase"

HAnimHumanoid4.joints.append(HAnimJoint94)
HAnimJoint95 = x3d.HAnimJoint()
HAnimJoint95.USE = "hanim_l_shoulder"

HAnimHumanoid4.joints.append(HAnimJoint95)
HAnimJoint96 = x3d.HAnimJoint()
HAnimJoint96.USE = "hanim_l_elbow"

HAnimHumanoid4.joints.append(HAnimJoint96)
HAnimJoint97 = x3d.HAnimJoint()
HAnimJoint97.USE = "hanim_l_wrist"

HAnimHumanoid4.joints.append(HAnimJoint97)
HAnimJoint98 = x3d.HAnimJoint()
HAnimJoint98.USE = "hanim_r_shoulder"

HAnimHumanoid4.joints.append(HAnimJoint98)
HAnimJoint99 = x3d.HAnimJoint()
HAnimJoint99.USE = "hanim_r_elbow"

HAnimHumanoid4.joints.append(HAnimJoint99)
HAnimJoint100 = x3d.HAnimJoint()
HAnimJoint100.USE = "hanim_r_wrist"

HAnimHumanoid4.joints.append(HAnimJoint100)
HAnimJoint101 = x3d.HAnimJoint()
HAnimJoint101.USE = "hanim_l_hip"

HAnimHumanoid4.joints.append(HAnimJoint101)
HAnimJoint102 = x3d.HAnimJoint()
HAnimJoint102.USE = "hanim_l_knee"

HAnimHumanoid4.joints.append(HAnimJoint102)
HAnimJoint103 = x3d.HAnimJoint()
HAnimJoint103.USE = "hanim_l_ankle"

HAnimHumanoid4.joints.append(HAnimJoint103)
HAnimJoint104 = x3d.HAnimJoint()
HAnimJoint104.USE = "hanim_r_hip"

HAnimHumanoid4.joints.append(HAnimJoint104)
HAnimJoint105 = x3d.HAnimJoint()
HAnimJoint105.USE = "hanim_r_knee"

HAnimHumanoid4.joints.append(HAnimJoint105)
HAnimJoint106 = x3d.HAnimJoint()
HAnimJoint106.USE = "hanim_r_ankle"

HAnimHumanoid4.joints.append(HAnimJoint106)

Scene3.children.append(HAnimHumanoid4)
TimeSensor107 = x3d.TimeSensor()
TimeSensor107.DEF = "Clock"
TimeSensor107.cycleInterval = 2
TimeSensor107.loop = True

Scene3.children.append(TimeSensor107)
OrientationInterpolator108 = x3d.OrientationInterpolator()
OrientationInterpolator108.DEF = "LHipAnimator"
OrientationInterpolator108.key = [0,0.25,0.5,0.75,1]
OrientationInterpolator108.keyValue = [(1, 0, 0, -0.5),(1, 0, 0, 0),(1, 0, 0, 0.5),(1, 0, 0, 0),(1, 0, 0, -0.5)]

Scene3.children.append(OrientationInterpolator108)
OrientationInterpolator109 = x3d.OrientationInterpolator()
OrientationInterpolator109.DEF = "RHipAnimator"
OrientationInterpolator109.key = [0,0.25,0.5,0.75,1]
OrientationInterpolator109.keyValue = [(1, 0, 0, 0.5),(1, 0, 0, 0),(1, 0, 0, -0.5),(1, 0, 0, 0),(1, 0, 0, 0.5)]

Scene3.children.append(OrientationInterpolator109)
OrientationInterpolator110 = x3d.OrientationInterpolator()
OrientationInterpolator110.DEF = "KneeAnimator"
OrientationInterpolator110.key = [0,0.25,0.5,0.75,1]
OrientationInterpolator110.keyValue = [(1, 0, 0, 0),(1, 0, 0, 1.2),(1, 0, 0, 0),(1, 0, 0, 0),(1, 0, 0, 0)]

Scene3.children.append(OrientationInterpolator110)
OrientationInterpolator111 = x3d.OrientationInterpolator()
OrientationInterpolator111.DEF = "LShoulderAnimator"
OrientationInterpolator111.key = [0,0.25,0.5,0.75,1]
OrientationInterpolator111.keyValue = [(1, 0, 0, 0.4),(1, 0, 0, 0),(1, 0, 0, -0.4),(1, 0, 0, 0),(1, 0, 0, 0.4)]

Scene3.children.append(OrientationInterpolator111)
OrientationInterpolator112 = x3d.OrientationInterpolator()
OrientationInterpolator112.DEF = "RShoulderAnimator"
OrientationInterpolator112.key = [0,0.25,0.5,0.75,1]
OrientationInterpolator112.keyValue = [(1, 0, 0, -0.4),(1, 0, 0, 0),(1, 0, 0, 0.4),(1, 0, 0, 0),(1, 0, 0, -0.4)]

Scene3.children.append(OrientationInterpolator112)
OrientationInterpolator113 = x3d.OrientationInterpolator()
OrientationInterpolator113.DEF = "ElbowAnimator"
OrientationInterpolator113.key = [0,0.25,0.5,0.75,1]
OrientationInterpolator113.keyValue = [(1, 0, 0, -0.3),(1, 0, 0, -0.8),(1, 0, 0, -0.3),(1, 0, 0, -0.3),(1, 0, 0, -0.3)]

Scene3.children.append(OrientationInterpolator113)
ROUTE114 = x3d.ROUTE()
ROUTE114.fromNode = "Clock"
ROUTE114.fromField = "fraction_changed"
ROUTE114.toNode = "LHipAnimator"
ROUTE114.toField = "set_fraction"

Scene3.children.append(ROUTE114)
ROUTE115 = x3d.ROUTE()
ROUTE115.fromNode = "LHipAnimator"
ROUTE115.fromField = "value_changed"
ROUTE115.toNode = "hanim_l_hip"
ROUTE115.toField = "set_rotation"

Scene3.children.append(ROUTE115)
ROUTE116 = x3d.ROUTE()
ROUTE116.fromNode = "Clock"
ROUTE116.fromField = "fraction_changed"
ROUTE116.toNode = "RHipAnimator"
ROUTE116.toField = "set_fraction"

Scene3.children.append(ROUTE116)
ROUTE117 = x3d.ROUTE()
ROUTE117.fromNode = "RHipAnimator"
ROUTE117.fromField = "value_changed"
ROUTE117.toNode = "hanim_r_hip"
ROUTE117.toField = "set_rotation"

Scene3.children.append(ROUTE117)
ROUTE118 = x3d.ROUTE()
ROUTE118.fromNode = "Clock"
ROUTE118.fromField = "fraction_changed"
ROUTE118.toNode = "KneeAnimator"
ROUTE118.toField = "set_fraction"

Scene3.children.append(ROUTE118)
ROUTE119 = x3d.ROUTE()
ROUTE119.fromNode = "KneeAnimator"
ROUTE119.fromField = "value_changed"
ROUTE119.toNode = "hanim_l_knee"
ROUTE119.toField = "set_rotation"

Scene3.children.append(ROUTE119)
ROUTE120 = x3d.ROUTE()
ROUTE120.fromNode = "KneeAnimator"
ROUTE120.fromField = "value_changed"
ROUTE120.toNode = "hanim_r_knee"
ROUTE120.toField = "set_rotation"

Scene3.children.append(ROUTE120)
ROUTE121 = x3d.ROUTE()
ROUTE121.fromNode = "Clock"
ROUTE121.fromField = "fraction_changed"
ROUTE121.toNode = "LShoulderAnimator"
ROUTE121.toField = "set_fraction"

Scene3.children.append(ROUTE121)
ROUTE122 = x3d.ROUTE()
ROUTE122.fromNode = "LShoulderAnimator"
ROUTE122.fromField = "value_changed"
ROUTE122.toNode = "hanim_l_shoulder"
ROUTE122.toField = "set_rotation"

Scene3.children.append(ROUTE122)
ROUTE123 = x3d.ROUTE()
ROUTE123.fromNode = "Clock"
ROUTE123.fromField = "fraction_changed"
ROUTE123.toNode = "RShoulderAnimator"
ROUTE123.toField = "set_fraction"

Scene3.children.append(ROUTE123)
ROUTE124 = x3d.ROUTE()
ROUTE124.fromNode = "RShoulderAnimator"
ROUTE124.fromField = "value_changed"
ROUTE124.toNode = "hanim_r_shoulder"
ROUTE124.toField = "set_rotation"

Scene3.children.append(ROUTE124)
ROUTE125 = x3d.ROUTE()
ROUTE125.fromNode = "Clock"
ROUTE125.fromField = "fraction_changed"
ROUTE125.toNode = "ElbowAnimator"
ROUTE125.toField = "set_fraction"

Scene3.children.append(ROUTE125)
ROUTE126 = x3d.ROUTE()
ROUTE126.fromNode = "ElbowAnimator"
ROUTE126.fromField = "value_changed"
ROUTE126.toNode = "hanim_l_elbow"
ROUTE126.toField = "set_rotation"

Scene3.children.append(ROUTE126)
ROUTE127 = x3d.ROUTE()
ROUTE127.fromNode = "ElbowAnimator"
ROUTE127.fromField = "value_changed"
ROUTE127.toNode = "hanim_r_elbow"
ROUTE127.toField = "set_rotation"

Scene3.children.append(ROUTE127)

X3D0.Scene = Scene3
f = open("walking_man.new.python.x3d", mode="w", encoding="utf-8")
f.write(X3D0.XML())
f.close()
