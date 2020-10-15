// test 1 2 3 

/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Tue Oct 01 2019                                           */
/*    Description:  2D Arrays                                                 */
/*                  This program will print items contained in a 2D Array.    */
/*                  This program uses 2 nested for loops to print the         */
/*                  integers.                                                 */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // Declare a 2D Array with 3 rows and 5 columns
  int my_2DList[3][5] = {
      {0, 1, 2, 3, 4},
      {5, 6, 7, 8, 9},
      {10, 11, 12, 13, 14},
  };
  // Loop through all of the rows.
  for (int rowIndex = 0; rowIndex < 3; rowIndex++) {
    // For each row, loop through the columns
    for (int colIndex = 0; colIndex < 5; colIndex++) {
      // Print the value of each element
      Brain.Screen.print("%d", my_2DList[rowIndex][colIndex]);

      // If it is not the last element in the column, add a separator.
      if (colIndex < 4) {
        Brain.Screen.print(" - ");
      }
      wait(5, msec);
    }
    // After the current row is printed. Move the cursor to the next line.
    Brain.Screen.newLine();
    wait(5, msec);
  }
}
