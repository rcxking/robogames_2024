// Constants.scad
// Shared mechanical dimensions across the robot.
/*
 * From mechanical diagrams a VersaFrame piece has the
 * following dimensions:
 *
 * Length: Specified via argument
 * Width: 1 inch
 * Height: 2 inches
 *
 * Additionally 5/32" holes are drilled every 0.5 inch.
 */
vf_width_inch = 1.0;
vf_height_inch = 2.0;

// The inner walls are 1/10" thick
vf_wall_thickness_inch = 0.1;

// Conversion factor from inches to millimeters
inch_to_mm = 25.4;

// Converted units (Gazebo uses metric)
vf_width_mm = vf_width_inch * inch_to_mm;
vf_height_mm = vf_height_inch * inch_to_mm;
vf_wall_thickness_mm = vf_wall_thickness_inch * inch_to_mm;

hole_diameter_inch = 5.0/32.0;
hole_spacing_inch = 0.5;
first_hole_inch = hole_spacing_inch;

hole_diameter_mm = hole_diameter_inch * inch_to_mm;
hole_spacing_mm = hole_spacing_inch * inch_to_mm;
first_hole_mm = first_hole_inch * inch_to_mm;
