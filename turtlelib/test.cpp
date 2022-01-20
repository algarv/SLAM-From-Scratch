#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "rigid2d.hpp"


TEST_CASE("constructor_trans", "[transform]") { // Anna Garverick
    turtlelib::Vector2D v;
    v.x = 1;
    v.y = 2;
    
    double r = turtlelib::deg2rad(90);
    
    turtlelib::Transform2D T(v, r);

    turtlelib::Vector2D t_out = T.translation();
    double r_out = T.rotation();

    double d = turtlelib::rad2deg(r_out);

    REQUIRE(t_out.x == 1);
    REQUIRE(t_out.x == 1);
    REQUIRE(d == 90);
} 


// TEST_CASE( "Factorials are computed", "[factorial]" ) {
//     REQUIRE( Factorial(1) == 1 );
//     REQUIRE( Factorial(2) == 2 );
//     REQUIRE( Factorial(3) == 6 );
//     REQUIRE( Factorial(10) == 3628800 );
// }