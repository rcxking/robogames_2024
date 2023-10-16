/*
 * VersaFrame.scad
 *
 * Module representing a single VersaFrame piece.  Units
 * are in inches per Vex mechanical diagrams.
 */
 
// Default length of a VersaFrame piece is 59"
module versaframe(length=59.0) {
  /*
   * From mechanical diagrams a VersaFrame piece has the
   * following dimensions:
   *
   * Length: Specified via argument
   * Width: 1 inch
   * Height: 2 inches
   *
   * Additionally 5/32" holes are drilled 
   */
  vf_length_inch = length;
  vf_width_inch = 1.0;
  vf_height_inch = 2.0;
  
  // The inner walls are 1/10" thick
  vf_wall_thickness_inch = 0.1;
  
  // Conversion factor from inches to millimeters
  inch_to_mm = 25.4;
  
  // Converted units (Gazebo uses metric)
  vf_length_mm = vf_length_inch * inch_to_mm;
  vf_width_mm = vf_width_inch * inch_to_mm;
  vf_height_mm = vf_height_inch * inch_to_mm;
  vf_wall_thickness_mm = vf_wall_thickness_inch * inch_to_mm;
  
  /*
   * 5/32" holes are evenly spaced every 0.5 inch.  The
   * holes are centered around these 0.5 inch marks.
   */
  hole_diameter_inch = 5.0/32.0;
  hole_spacing_inch = 0.5;
  first_hole_inch = hole_spacing_inch;
  last_hole_inch = length - hole_spacing_inch;

  hole_diameter_mm = hole_diameter_inch * inch_to_mm;
  hole_spacing_mm = hole_spacing_inch * inch_to_mm;
  first_hole_mm = first_hole_inch * inch_to_mm;
  last_hole_mm = last_hole_inch * inch_to_mm;

  difference() {
    difference() {
      cube([vf_length_mm, vf_width_mm, vf_height_mm]);
      
      /*
       * Need to translate the inner wall a little longer
       * than the length of the frame due to boundary 
       * conditions.
       */
      translate([-0.01, vf_wall_thickness_mm, vf_wall_thickness_mm]) {
        cube([vf_length_mm + 0.02, vf_width_mm - vf_wall_thickness_mm * 2, vf_height_mm - vf_wall_thickness_mm * 2]);
      }
    }
    
    for(x = [first_hole_mm:hole_spacing_mm:last_hole_mm]) {
      translate([x, vf_width_mm/2, -0.01]) {
        cylinder(h = vf_height_mm + 2 * 0.01, d=hole_diameter_mm, center=false);
      }
    }
  }
}