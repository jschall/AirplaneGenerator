from WingSectionGenerator import *
import cadquery as cq
from math import *

skin_thickness = 0.8

aileron_start_chord = 0.8
aileron_length = 0.6
aileron_start_norm_span = 0.2
aileron_pin_radius = 0.5
aileron_hinge_thickness = 2.
aileron_hinge_clearance = 0.6
aileron_chordwise_clearance = 0.6
aileron_pin_depth = 5.
aileron_max_deflection = radians(60)

reinforcement_interval = 40
reinforcement_width = 0.4

wing = WingSectionGenerator(taper_ratio=1.0, washout=0, dihedral=0, sweep=0)
airfoil = wing.airfoil


Z = np.linspace(0,wing.wing_length,2)

def get_section(Z1,Z2):
    wp = cq.Workplane('XY', origin=(0,0,Z1)).polyline(wing.getAirfoil(Z1)).close()
    wp = wp.workplane(offset=Z2-Z1)
    wp = wp.polyline(wing.getAirfoil(Z2)).close()
    wp = wp.loft(combine=True, ruled=True)
    return wp

def get_inside_section(Z1,Z2):
    wp = cq.Workplane('XY', origin=(0,0,Z1)).polyline(wing.getOffsetAirfoil(Z1,-skin_thickness)).close()
    wp = wp.workplane(offset=Z2-Z1)
    wp = wp.polyline(wing.getOffsetAirfoil(Z2,-skin_thickness)).close()
    wp = wp.loft(combine=True, ruled=True)
    return wp

def generate_wing_inner_volume():
    inside_wing_sections = [get_inside_section(Z1,Z2) for Z1,Z2 in zip(Z[:-1], Z[1:])]
    inside_wing = inside_wing_sections[0]
    for section in inside_wing_sections[1:]:
        inside_wing = inside_wing.union(section)
    return inside_wing

def generate_wing_outer_volume():
    outside_wing_sections = [get_section(Z1,Z2) for Z1,Z2 in zip(Z[:-1], Z[1:])]

    outside_wing = outside_wing_sections[0]
    for section in outside_wing_sections[1:]:
        outside_wing = outside_wing.union(section)
    return outside_wing

def get_hinge_start_workplane(offset=0):
    hinge_line_start_Z = aileron_start_norm_span*wing.wing_length-offset
    hinge_line_end_Z = (aileron_start_norm_span+aileron_length)*wing.wing_length+offset

    hinge_line_start = np.append(wing.getCamberPoint(aileron_start_chord, hinge_line_start_Z), [hinge_line_start_Z])
    hinge_line_end = np.append(wing.getCamberPoint(aileron_start_chord, hinge_line_end_Z), [hinge_line_end_Z])

    hinge_line_normal = (hinge_line_end-hinge_line_start)/np.linalg.norm(hinge_line_end-hinge_line_start)
    xdir = np.asarray((1.,0.,0.))
    xdir = xdir-hinge_line_normal*xdir.dot(hinge_line_normal)
    return cq.Workplane(cq.Plane(tuple(hinge_line_start), tuple(xdir), tuple(hinge_line_normal)),origin=tuple(hinge_line_start))

def generate_hinge_pin():
    wp = get_hinge_start_workplane(aileron_pin_depth)
    return wp.circle(aileron_pin_radius).extrude(aileron_length*wing.wing_length + 2*aileron_pin_depth)

def generate_aileron_spanwise_cut():
    cut_inside_radius = aileron_pin_radius+aileron_hinge_thickness
    cut_outside_radius = aileron_pin_radius+aileron_hinge_thickness+aileron_hinge_clearance
    hinge_line_start_Z = aileron_start_norm_span*wing.wing_length

    hinge_begin_point_2d = wing.getCamberPoint(aileron_start_chord, hinge_line_start_Z)

    wp = get_hinge_start_workplane()

    top_fore,bot_aft = wing.getLineAcrossCamber(aileron_start_chord, aileron_max_deflection/2,hinge_line_start_Z)
    top_aft,bot_fore = wing.getLineAcrossCamber(aileron_start_chord, -aileron_max_deflection/2,hinge_line_start_Z)
    top_fore -= hinge_begin_point_2d
    bot_fore -= hinge_begin_point_2d
    top_aft -= hinge_begin_point_2d
    bot_aft -= hinge_begin_point_2d

    top_fore_norm = top_fore/np.linalg.norm(top_fore)
    top_aft_norm = top_aft/np.linalg.norm(top_aft)
    bot_fore_norm = bot_fore/np.linalg.norm(bot_fore)
    bot_aft_norm = bot_aft/np.linalg.norm(bot_aft)
    wp = wp.pushPoints([tuple(bot_aft_norm*cut_inside_radius)])
    wp = wp.threePointArc((-cut_inside_radius,0),tuple(top_aft_norm*cut_inside_radius))
    wp = wp.lineTo(*(top_aft*2))
    wp = wp.radiusArc(tuple(top_fore*2), -np.linalg.norm(top_fore)*2)
    wp = wp.lineTo(*(top_fore_norm*cut_outside_radius))
    wp = wp.threePointArc((-cut_outside_radius,0), tuple(bot_fore_norm*cut_outside_radius))
    wp = wp.lineTo(*(bot_fore*2))
    wp = wp.radiusArc(tuple(bot_aft*2), -np.linalg.norm(top_fore)*2)

    wp = wp.close().extrude(aileron_length*wing.wing_length)

    return wp

def generate_aileron_chordwise_cuts():
    cut_outside_radius = aileron_pin_radius+aileron_hinge_thickness+aileron_hinge_clearance
    hinge_line_start_Z = aileron_start_norm_span*wing.wing_length

    hinge_begin_point_2d = wing.getCamberPoint(aileron_start_chord, hinge_line_start_Z)

    wp = get_hinge_start_workplane()

    top_fore,bot_aft = wing.getLineAcrossCamber(aileron_start_chord, aileron_max_deflection/2,hinge_line_start_Z)
    top_aft,bot_fore = wing.getLineAcrossCamber(aileron_start_chord, -aileron_max_deflection/2,hinge_line_start_Z)
    top_fore -= hinge_begin_point_2d
    bot_fore -= hinge_begin_point_2d
    top_aft -= hinge_begin_point_2d
    bot_aft -= hinge_begin_point_2d

    top_fore_norm = top_fore/np.linalg.norm(top_fore)
    top_aft_norm = top_aft/np.linalg.norm(top_aft)
    bot_fore_norm = bot_fore/np.linalg.norm(bot_fore)
    bot_aft_norm = bot_aft/np.linalg.norm(bot_aft)

    wp = wp.pushPoints([tuple(bot_fore*2)])
    wp = wp.lineTo(*bot_fore_norm*cut_outside_radius)
    wp = wp.threePointArc((-cut_outside_radius,0),tuple(top_fore_norm*cut_outside_radius))
    wp = wp.lineTo(*(top_fore*2))
    wp = wp.threePointArc((wing.root_chord/2,0), tuple(bot_fore*2))

    wp = wp.close().extrude(aileron_chordwise_clearance)
    wp = wp.add(wp.translate((0,0,aileron_length*wing.wing_length-aileron_chordwise_clearance)))

    return wp

def generate_reinforcements():
    base_reinforcement = cq.Workplane('XY', origin=(0,0,-wing.wing_length/2)).box(wing.root_chord*10,wing.root_chord*10,reinforcement_width)

    reinforcements = base_reinforcement.rotate((0,0,0),(0,1,0),60) + base_reinforcement.rotate((0,0,0),(0,1,0),-60)
    for i in range(1,int(2*wing.wing_length/reinforcement_interval)):
        reinforcements += base_reinforcement.translate((0,0,reinforcement_interval*i)).rotate((0,0,0),(0,1,0),60)
        reinforcements += base_reinforcement.translate((0,0,reinforcement_interval*i)).rotate((0,0,0),(0,1,0),-60)

    return reinforcements

# generate the wing volume and wing internal volume
wing_outer_volume = generate_wing_outer_volume()
wing_inner_volume = wing_outer_volume-wing_outer_volume.shell(-skin_thickness)
wing_inner_volume = generate_wing_inner_volume()

# cut hinge pin out
hinge_pin = generate_hinge_pin()
wing_outer_volume -= hinge_pin
wing_inner_volume -= hinge_pin+hinge_pin.shell(skin_thickness)

# cut spanwise_cut
spanwise_cut = generate_aileron_spanwise_cut()
wing_outer_volume -= spanwise_cut
wing_inner_volume -= spanwise_cut+spanwise_cut.shell(skin_thickness)

# cut chordwise cut
chordwise_cuts = generate_aileron_chordwise_cuts()
wing_outer_volume -= chordwise_cuts
for solid in chordwise_cuts.solids().all():
    wing_inner_volume -= solid+solid.shell(skin_thickness)

featured_wing = wing_outer_volume-(generate_reinforcements() & wing_inner_volume)

del solid
del chordwise_cuts
del spanwise_cut
#del hinge_pin
#del wing_outer_volume
