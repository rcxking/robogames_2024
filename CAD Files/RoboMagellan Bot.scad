/*
 * RoboMagellan Bot.scad
 */
 
use <VersaFrame.scad>

/*
 * Number of faces to use.  Higher number = smoother
 * faces at the cost of increased rendering time.
 */
$fn = 25;

versaframe();

translate([0, 64, 0]) {
  versaframe(12);
}