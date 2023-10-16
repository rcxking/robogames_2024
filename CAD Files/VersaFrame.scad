/*
 * VersaFrame.scad
 *
 * Module representing a single VersaFrame piece.  Units
 * are in inches per Vex mechanical diagrams.
 */
 
include <Constants.scad>
 
// Helper module to render the top and bottom walls
module versaframe_top_bottom(vf_wall_thickness_mm,
                             vf_length_mm, 
                             vf_width_mm,
                             first_hole_mm,
                             hole_spacing_mm,
                             last_hole_mm,
                             hole_diameter_mm) {
  linear_extrude(vf_wall_thickness_mm) {
    difference() {
      square([vf_length_mm, vf_width_mm]);
        for (x = [first_hole_mm:hole_spacing_mm:last_hole_mm]) {
        translate([x, vf_width_mm/2]) {
          circle(d=hole_diameter_mm);
        }
      }
    }
  }
}

// Helper module to render the side walls
module versaframe_sidewall(vf_height_mm,
                           vf_wall_thickness_mm,
                           vf_length_mm) {
  linear_extrude(vf_height_mm - vf_wall_thickness_mm) {
    square([vf_length_mm, vf_wall_thickness_mm]);
  }
}
 
// Default length of a VersaFrame piece is 59"
module versaframe(length=59.0) {
  vf_length_inch = length;
  vf_length_mm = vf_length_inch * inch_to_mm;
  last_hole_inch = length - hole_spacing_inch;
  last_hole_mm = last_hole_inch * inch_to_mm;
  
  // Rendering a beam with many holes takes time, so to
  // decrease rendering time (standard 59" beam at 100
  // faces takes 3 minutes, 26 seconds) we can represent
  // the top and bottom of the beam as linearly-extruded
  // rectangles with holes.  The side walls can also be
  // represented with a pair of linearly-extruded narrow
  // rectangles.
  
  union() {
    // Bottom piece
    versaframe_top_bottom(vf_wall_thickness_mm,
                          vf_length_mm,
                          vf_width_mm,
                          first_hole_mm,
                          hole_spacing_mm,
                          last_hole_mm,
                          hole_diameter_mm);
    
    // Top piece
    translate([0, 0, vf_height_mm - vf_wall_thickness_mm]) {
      versaframe_top_bottom(vf_wall_thickness_mm,
                            vf_length_mm,
                            vf_width_mm,
                            first_hole_mm,
                            hole_spacing_mm,
                            last_hole_mm,
                            hole_diameter_mm);
    }
    
    // Left wall
    translate([0, vf_width_mm - vf_wall_thickness_mm, vf_wall_thickness_mm]) {
      versaframe_sidewall(vf_height_mm,
                          vf_wall_thickness_mm,
                          vf_length_mm);
    }
    
    // Right wall
    versaframe_sidewall(vf_height_mm,
                        vf_wall_thickness_mm,
                        vf_length_mm);
  }
}