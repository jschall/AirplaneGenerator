from WingSectionGenerator import *
import cadquery as cq

skin_thickness = 0.8

aileron_start_chord = 0.8
aileron_length = 0.6
aileron_start_norm_span = 0.2
aileron_hinge_radius = 1.
aileron_pin_depth = 5.

section_generator = WingSectionGenerator()
airfoil = section_generator.airfoil

Z = np.linspace(0,section_generator.wing_length,2)

def get_section(Z1,Z2):
    wp = cq.Workplane('XY', origin=(0,0,Z1)).polyline(section_generator.getAirfoil(Z1)).close()
    wp = wp.workplane(offset=Z2-Z1)
    wp = wp.polyline(section_generator.getAirfoil(Z2)).close().offset2D(skin_thickness, 'intersection')
    wp = wp.loft(combine=True)
    return wp

def get_inside_section(Z1,Z2):
    wp = cq.Workplane('XY', origin=(0,0,Z1)).polyline(section_generator.getAirfoil(Z1)).close()
    wp = wp.workplane(offset=Z2-Z1)
    wp = wp.polyline(section_generator.getAirfoil(Z2)).close().offset2D(skin_thickness, 'intersection')
    wp = wp.loft(combine=True)
    return wp

def generate_wing_internal_volume():
    inside_wing_sections = [get_inside_section(Z1,Z2) for Z1,Z2 in zip(Z[:-1], Z[1:])]
    inside_wing = inside_wing_sections[0]
    for section in inside_wing_sections[1:]:
        inside_wing = inside_wing.union(section)
    return inside_wing

def generate_wing_volume():
    outside_wing_sections = [get_section(Z1,Z2) for Z1,Z2 in zip(Z[:-1], Z[1:])]

    outside_wing = outside_wing_sections[0]
    for section in outside_wing_sections[1:]:
        outside_wing = outside_wing.union(section)
    return outside_wing

def generate_hinge():
    hinge_line_start_Z = aileron_start_norm_span*section_generator.wing_length-aileron_pin_depth
    hinge_line_end_Z = (aileron_start_norm_span+aileron_length)*section_generator.wing_length+aileron_pin_depth

    hinge_line_start = np.append(section_generator.getCamberPoint(aileron_start_chord, hinge_line_start_Z), [hinge_line_start_Z])
    hinge_line_end = np.append(section_generator.getCamberPoint(aileron_start_chord, hinge_line_end_Z), [hinge_line_end_Z])

    hinge_line_normal = (hinge_line_end-hinge_line_start)/np.linalg.norm(hinge_line_end-hinge_line_start)
    xdir = np.asarray((1.,0.,0.))
    xdir = xdir-hinge_line_normal*xdir.dot(hinge_line_normal)
    hinge_start_plane = cq.Plane(tuple(hinge_line_start), tuple(xdir), tuple(hinge_line_normal))

    return cq.Workplane(hinge_start_plane,origin=tuple(hinge_line_start)).circle(aileron_hinge_radius).extrude(aileron_length*section_generator.wing_length)

hinge = generate_hinge()
wing_internal_volume = generate_wing_internal_volume()
wing_volume = generate_wing_volume()
