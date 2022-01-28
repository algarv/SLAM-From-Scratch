// #define CATCH_CONFIG_MAIN
#include "catch_ros/catch.hpp"
#include "turtlelib/rigid2d.hpp"
#include <sstream>

TEST_CASE("constructor_all", "[transform]") { // Anna Garverick
    turtlelib::Vector2D v;
    v.x = 1;
    v.y = 2;
    
    double r = turtlelib::deg2rad(90);
    
    turtlelib::Transform2D T(v, r);

    turtlelib::Vector2D t_out = T.translation();
    double r_out = T.rotation();

    double d = turtlelib::rad2deg(r_out);

    REQUIRE(t_out.x == 1);
    REQUIRE(t_out.y == 2);
    REQUIRE(d == 90);
} 

TEST_CASE("constructor_trans", "[transform]") { // Anna Garverick
    turtlelib::Vector2D v;
    v.x = 1;
    v.y = 2;
        
    turtlelib::Transform2D T(v);

    turtlelib::Vector2D t_out = T.translation();
    double r_out = T.rotation();

    double d = turtlelib::rad2deg(r_out);

    REQUIRE(t_out.x == 1);
    REQUIRE(t_out.y == 2);
    REQUIRE(d == 0);
}

TEST_CASE("constructor_rot", "[transform]") { // Anna Garverick
    double r = turtlelib::deg2rad(90);
    
    turtlelib::Transform2D T(r);

    turtlelib::Vector2D t_out = T.translation();
    double r_out = T.rotation();

    double d = turtlelib::rad2deg(r_out);

    REQUIRE(t_out.x == 0);
    REQUIRE(t_out.y == 0);
    REQUIRE(d == 90);
}

TEST_CASE("inv", "[inverse]") { // Anna Garverick
    turtlelib::Vector2D v;
    v.x = 1;
    v.y = 2;
    
    double r = turtlelib::PI/4;

    turtlelib::Transform2D T(v,r);

    turtlelib::Transform2D T_inv(0);
    T_inv = T.inv();

    turtlelib::Vector2D t_out = T_inv.translation();
    double r_out = T_inv.rotation();

    REQUIRE(t_out.x == Approx(-2.121).margin(.01));
    REQUIRE(t_out.y == Approx(-0.7071).margin(.01));
    REQUIRE(r_out == Approx(-1*turtlelib::PI/4).margin(.01));
}

TEST_CASE("trans", "[translation]") { //Anna Garverick
    turtlelib::Vector2D v;
    v.x = 5;
    v.y = 10;
        
    turtlelib::Transform2D T(v);

    turtlelib::Vector2D t_out = T.translation();

    REQUIRE(t_out.x == 5);
    REQUIRE(t_out.y == 10);
}

TEST_CASE("rot", "[rotation]") { //Anna Garverick
    double r = turtlelib::deg2rad(33);
    
    turtlelib::Transform2D T(r);

    double r_out = T.rotation();

    double d = turtlelib::rad2deg(r_out);

    REQUIRE(d == Approx(33).margin(.001));
}

TEST_CASE("normalize"){ // Cody Nichoson
    turtlelib::Vector2D v_in, v_out;
    v_in.x = 1; v_in.y = 1;
    v_out = normalize(v_in);
    REQUIRE(v_out.x == Approx(0.7071067812));
    REQUIRE(v_out.y == Approx(0.7071067812));
}

TEST_CASE("istream Vector input","[Vector2D]"){// James Avtges
    std::stringstream bracket;
    turtlelib::Vector2D bracketV;
    bracket.str("[1 1]");
    std::stringstream number;
    turtlelib::Vector2D numberV;
    number.str("1 1");
    bracket >> bracketV;
    number >> numberV;
    REQUIRE(bracketV.x == 1);
    REQUIRE(numberV.x == 1);
}

TEST_CASE("ostream Vector output","[Vector2D]"){// James Avtges
    std::stringstream vectorOut;
    turtlelib::Vector2D vector;
    vector.x = 9;
    vector.y = 1;

    vectorOut << vector;

    REQUIRE(vectorOut.str() == "[9 1]\n");
}

TEST_CASE("istream Twist input","[Twist2D]"){ // James Avtges
    std::stringstream bracket;
    turtlelib::Twist2D bracketT;
    bracket.str("[1 2 3]");
    std::stringstream number;
    turtlelib::Twist2D numberT;
    number.str("1 2 3");
    bracket >> bracketT;
    number >> numberT;
    REQUIRE(bracketT.vx == 2);
    REQUIRE(numberT.vx == 2);
}

TEST_CASE("ostream Twist output","[Twist2D]"){// James Avtges
    std::stringstream twistOut;
    turtlelib::Twist2D twist;
    twist.w = 10;
    twist.vx = 5;
    twist.vy = 4;

    twistOut << twist;

    REQUIRE(twistOut.str() == "[10 5 4]\n");
}

TEST_CASE("istream Transform input","[Transform2D]"){ // James Avtges
    std::stringstream transform;
    turtlelib::Transform2D T(0);
    transform.str("deg: 80 x: 2 y: 4\n");
    transform >> T;

    turtlelib::Vector2D translation = T.translation();
    double rotation = T.rotation();
    REQUIRE(translation.x == 2);
    REQUIRE(translation.y == 4);
    REQUIRE(rotation == Approx(turtlelib::deg2rad(80)).margin(.001));
}

TEST_CASE("ostream Transform output","[Transform2D]"){//James Avtges
    std::stringstream transformOut;
    turtlelib::Vector2D trans;
    trans.x = 3.2;
    trans.y = 4;
    double rot = turtlelib::deg2rad(6.1);
    turtlelib::Transform2D T(trans,rot);

    transformOut << T;

    REQUIRE(transformOut.str() == "deg: 6.1 x: 3.2 y: 4\n");
} 

static const double epsilon = 1.0e-12;
using namespace turtlelib;

TEST_CASE("Transform2D::operator*=, Self Reference", "[Transform2D]") //Ryan King Shepard
{
    //Build objects
    Transform2D T_id{0};
    Vector2D vec_id{0,0};

    //Perform functions
    T_id *= T_id;

    //Grab results
    Vector2D res_vec = T_id.translation();
    double res_ang = T_id.rotation();

    // Tests
    REQUIRE(res_vec.x == Approx( vec_id.x ).margin(epsilon));
    REQUIRE(res_vec.y == Approx( vec_id.y ).margin(epsilon));
    REQUIRE(res_ang == Approx( 0.0 ).margin(epsilon));
}

TEST_CASE("Transform2D::operator*=, Identity Mul", "[Transform2D]") //Ryan King Shepard
{
    //Build objects
    Transform2D T_id{0};
    Transform2D T_id2{0};
    Vector2D vec_id{0,0};

    //Perform functions
    T_id *= T_id2;

    //Grab results
    Vector2D res_vec = T_id.translation();
    double res_ang = T_id.rotation();

    // Tests
    REQUIRE(res_vec.x == Approx( vec_id.x ).margin(epsilon));
    REQUIRE(res_vec.y == Approx( vec_id.y ).margin(epsilon));
    REQUIRE(res_ang == Approx( 0.0 ).margin(epsilon));
}

TEST_CASE("Transform2D::operator*=, Cancelling Rotations", "[Transform2D]") //Ryan King Shepard
{
    //Build objects
    Transform2D T_a{-PI/4};
    Transform2D T_b{PI/4};
    Vector2D vec_id{0,0};

    //Perform functions
    T_a *= T_b;

    //Grab results
    Vector2D res_vec = T_a.translation();
    double res_ang = T_a.rotation();

    // Tests
    REQUIRE(res_vec.x == Approx( vec_id.x ).margin(epsilon));
    REQUIRE(res_vec.y == Approx( vec_id.y ).margin(epsilon));
    REQUIRE(res_ang == Approx( 0.0 ).margin(epsilon));
}

TEST_CASE("Transform2D::operator*=, Cancelling Transitions", "[Transform2D]") //Ryan King Shepard
{
    //Build objects
    Vector2D vec_a{1,2};
    Vector2D vec_b{-1,-2};
    Transform2D T_a{vec_a};
    Transform2D T_b{vec_b};
    Vector2D vec_id{0,0};

    //Perform functions
    T_a *= T_b;

    //Grab results
    Vector2D res_vec = T_a.translation();
    double res_ang = T_a.rotation();

    // Tests
    REQUIRE(res_vec.x == Approx( vec_id.x ).margin(epsilon));
    REQUIRE(res_vec.y == Approx( vec_id.y ).margin(epsilon));
    REQUIRE(res_ang == Approx( 0.0 ).margin(epsilon));
}

TEST_CASE("Transform2D::operator*=, Only Transitions", "[Transform2D]") //Ryan King Shepard
{
    //Build objects
    Vector2D vec_a{2,2};
    Vector2D vec_b{1,3};
    Transform2D T_a{vec_a};
    Transform2D T_b{vec_b};
    Vector2D vec_id{3,5};

    //Perform functions
    T_a *= T_b;

    //Grab results
    Vector2D res_vec = T_a.translation();
    double res_ang = T_a.rotation();

    // Tests
    REQUIRE(res_vec.x == Approx( vec_id.x ).margin(epsilon));
    REQUIRE(res_vec.y == Approx( vec_id.y ).margin(epsilon));
    REQUIRE(res_ang == Approx( 0.0 ).margin(epsilon));
}

TEST_CASE("Transform2D::operator*=, Basic Transformation", "[Transform2D]") //Ryan King Shepard
{
    //Build objects
    Vector2D vec_a{1,2};
    double ang_a = PI/4;
    Vector2D vec_b{2,3};
    double ang_b = PI/3;
    Transform2D T_a{vec_a, ang_a};
    Transform2D T_b{vec_b, ang_b};

    // Answer
    Vector2D vec_ans{0.292893,5.53553};
    double ang_ans = 1.8326;

    //Perform functions
    T_a *= T_b;

    //Grab results
    Vector2D res_vec = T_a.translation();
    double res_ang = T_a.rotation();

    // Tests
    REQUIRE(res_vec.x == Approx( vec_ans.x ).margin(epsilon));
    REQUIRE(res_vec.y == Approx( vec_ans.y ).margin(epsilon));
    REQUIRE(res_ang == Approx( ang_ans ).margin(epsilon));
}

TEST_CASE("Transform2D::operator*=, Big Rotation", "[Transform2D]") //Ryan King Shepard
{
    //Build objects
    Vector2D vec_a{1,2};
    double ang_a = 4.88692;
    Vector2D vec_b{-2,3};
    double ang_b = 1.74533;
    Transform2D T_a{vec_a, ang_a};
    Transform2D T_b{vec_b, ang_b};

    // Answer
    Vector2D vec_ans{3.60713, 4.49056};
    double ang_ans = 0.349066;

    //Perform functions
    T_a *= T_b;

    //Grab results
    Vector2D res_vec = T_a.translation();
    double res_ang = std::fmod(T_a.rotation(), 2.0*PI);

    // Tests
    REQUIRE(res_vec.x == Approx( vec_ans.x ).margin(epsilon));
    REQUIRE(res_vec.y == Approx( vec_ans.y ).margin(epsilon));
    REQUIRE(res_ang == Approx( ang_ans ).margin(epsilon));
}


/// TEST operator* ///

TEST_CASE("Transform2D::operator*, Identity Mul", "[Transform2D]") //Ryan King Shepard
{
    //Build objects
    Transform2D T_id{0};
    Transform2D T_id2{0};
    Vector2D vec_id{0,0};

    //Perform functions
    Transform2D T_id3 = T_id * T_id2;

    //Grab results
    Vector2D res_vec = T_id3.translation();
    double res_ang = T_id3.rotation();

    // Tests
    REQUIRE(res_vec.x == Approx( vec_id.x ).margin(epsilon));
    REQUIRE(res_vec.y == Approx( vec_id.y ).margin(epsilon));
    REQUIRE(res_ang == Approx( 0.0 ).margin(epsilon));
}

TEST_CASE("Transform2D::operator*, Cancelling Rotations", "[Transform2D]") //Ryan King Shepard
{
    //Build objects
    Transform2D T_a{-PI/4};
    Transform2D T_b{PI/4};
    Vector2D vec_id{0,0};

    //Perform functions
    Transform2D T_c = T_a * T_b;

    //Grab results
    Vector2D res_vec = T_c.translation();
    double res_ang = T_c.rotation();

    // Tests
    REQUIRE(res_vec.x == Approx( vec_id.x ).margin(epsilon));
    REQUIRE(res_vec.y == Approx( vec_id.y ).margin(epsilon));
    REQUIRE(res_ang == Approx( 0.0 ).margin(epsilon));
}

TEST_CASE("Transform2D::operator*, Cancelling Transitions", "[Transform2D]") //Ryan King Shepard
{
    //Build objects
    Vector2D vec_a{1,2};
    Vector2D vec_b{-1,-2};
    Transform2D T_a{vec_a};
    Transform2D T_b{vec_b};
    Vector2D vec_id{0,0};

    //Perform functions
    Transform2D T_c = T_a * T_b;

    //Grab results
    Vector2D res_vec = T_c.translation();
    double res_ang = T_c.rotation();

    // Tests
    REQUIRE(res_vec.x == Approx( vec_id.x ).margin(epsilon));
    REQUIRE(res_vec.y == Approx( vec_id.y ).margin(epsilon));
    REQUIRE(res_ang == Approx( 0.0 ).margin(epsilon));
}

TEST_CASE("Transform2D::operator*, Only Transitions", "[Transform2D]") //Ryan King Shepard
{
    //Build objects
    Vector2D vec_a{2,2};
    Vector2D vec_b{1,3};
    Transform2D T_a{vec_a};
    Transform2D T_b{vec_b};
    Vector2D vec_id{3,5};

    //Perform functions
    Transform2D T_c = T_a * T_b;

    //Grab results
    Vector2D res_vec = T_c.translation();
    double res_ang = T_c.rotation();

    // Tests
    REQUIRE(res_vec.x == Approx( vec_id.x ).margin(epsilon));
    REQUIRE(res_vec.y == Approx( vec_id.y ).margin(epsilon));
    REQUIRE(res_ang == Approx( 0.0 ).margin(epsilon));
}

TEST_CASE("Transform2D::operator*, Basic Transformation", "[Transform2D]") //Ryan King Shepard
{
    //Build objects
    Vector2D vec_a{1,2};
    double ang_a = PI/4;
    Vector2D vec_b{2,3};
    double ang_b = PI/3;
    Transform2D T_a{vec_a, ang_a};
    Transform2D T_b{vec_b, ang_b};

    // Answer
    Vector2D vec_ans{0.292893,5.53553};
    double ang_ans = 1.8326;

    //Perform functions
    Transform2D T_c = T_a * T_b;

    //Grab results
    Vector2D res_vec = T_c.translation();
    double res_ang = T_c.rotation();

    // Tests
    REQUIRE(res_vec.x == Approx( vec_ans.x ).margin(epsilon));
    REQUIRE(res_vec.y == Approx( vec_ans.y ).margin(epsilon));
    REQUIRE(res_ang == Approx( ang_ans ).margin(epsilon));
}

TEST_CASE("Transform2D::operator*, Big Rotation", "[Transform2D]") //Ryan King Shepard
{
    //Build objects
    Vector2D vec_a{1,2};
    double ang_a = 4.88692;
    Vector2D vec_b{-2,3};
    double ang_b = 1.74533;
    Transform2D T_a{vec_a, ang_a};
    Transform2D T_b{vec_b, ang_b};

    // Answer
    Vector2D vec_ans{3.60713, 4.49056};
    double ang_ans = 0.349066;

    //Perform functions
    Transform2D T_c = T_a * T_b;

    //Grab results
    Vector2D res_vec = T_c.translation();
    double res_ang = std::fmod(T_c.rotation(), 2.0*PI);

    // Tests
    REQUIRE(res_vec.x == Approx( vec_ans.x ).margin(epsilon));
    REQUIRE(res_vec.y == Approx( vec_ans.y ).margin(epsilon));
    REQUIRE(res_ang == Approx( ang_ans ).margin(epsilon));
}

/// TEST operator()(Vector2D) ///

TEST_CASE("Transform2D::operator(Vector), Idenity Transform", "[Transform2D]") //Ryan King Shepard
{
    //Build objects
    Transform2D T_ab{0};
    Vector2D vec_b{1.0, 1.0};

    // Answer
    Vector2D vec_ans{1.0, 1.0};

    //Perform functions
    Vector2D res_vec = T_ab(vec_b);

    // Tests
    REQUIRE(res_vec.x == Approx( vec_ans.x ).margin(epsilon));
    REQUIRE(res_vec.y == Approx( vec_ans.y ).margin(epsilon));
}

TEST_CASE("Transform2D::operator(Vector), Simple Rotation", "[Transform2D]") //Ryan King Shepard
{
    //Build objects
    Transform2D T_ab{PI/4};
    Vector2D vec_b{1.0, 1.0};

    // Answer
    Vector2D vec_ans{0.0, 1.414213};

    //Perform functions
    Vector2D res_vec = T_ab(vec_b);
    
    // Tests
    REQUIRE(res_vec.x == Approx( vec_ans.x ).margin(epsilon));
    REQUIRE(res_vec.y == Approx( vec_ans.y ).margin(epsilon));
}

TEST_CASE("Transform2D::operator(Vector), Simple Translation", "[Transform2D]") //Ryan King Shepard
{
    //Build objects
    Vector2D vec_a{2.0, -2.0};
    Transform2D T_ab{vec_a};
    Vector2D vec_b{1.0, 1.0};

    // Answer
    Vector2D vec_ans{3.0, -1.0};

    //Perform functions
    Vector2D res_vec = T_ab(vec_b);
    
    // Tests
    REQUIRE(res_vec.x == Approx( vec_ans.x ).margin(epsilon));
    REQUIRE(res_vec.y == Approx( vec_ans.y ).margin(epsilon));
}

TEST_CASE("Transform2D::operator(Vector), Simple Transformation", "[Transform2D]") //Ryan King Shepard
{
    //Build objects
    Vector2D vec_a{2.0, -2.0};
    Transform2D T_ab{vec_a, PI/4};
    Vector2D vec_b{1.0, 1.0};

    // Answer
    Vector2D vec_ans{2., -0.58578};

    //Perform functions
    Vector2D res_vec = T_ab(vec_b);
    
    // Tests
    REQUIRE(res_vec.x == Approx( vec_ans.x ).margin(epsilon));
    REQUIRE(res_vec.y == Approx( vec_ans.y ).margin(epsilon));
}

/// TEST operator()(Twist2D) ///

TEST_CASE("Transform2D::operator(Twist2D), Identity Transform", "[Transform2D]") //Ryan King Shepard
{
    //Build objects
    Transform2D T_ab{0};
    Twist2D twt_b{PI/6, 1.0, 2.0};

    // Answer
    Twist2D twt_ans{0.52359878, 1.0, 2.0};

    //Perform functions
    Twist2D res_twt = T_ab(twt_b);

    // Tests
    REQUIRE(res_twt.w == Approx ( twt_ans.w).margin(epsilon));
    REQUIRE(res_twt.vx == Approx( twt_ans.vx).margin(epsilon));
    REQUIRE(res_twt.vy == Approx( twt_ans.vy).margin(epsilon));
}

TEST_CASE("Transform2D::operator(Twist2D), Simple Rotation", "[Transform2D]") //Ryan King Shepard
{
    //Build objects
    Transform2D T_ab{PI/4};
    Twist2D twt_b{PI/6, 1.0, 2.0};

    // Answer
    Twist2D twt_ans{0.52359878, -0.70710678, 2.12132034};

    //Perform functions
    Twist2D res_twt = T_ab(twt_b);

    // Tests
    REQUIRE(res_twt.w == Approx ( twt_ans.w).margin(epsilon));
    REQUIRE(res_twt.vx == Approx( twt_ans.vx ).margin(epsilon));
    REQUIRE(res_twt.vy == Approx( twt_ans.vy ).margin(epsilon));
}

TEST_CASE("Transform2D::operator(Twist2D), Simple Translation", "[Transform2D]") //Ryan King Shepard
{
    //Build objects
    Vector2D vec_a{2.0, -2.0};
    Transform2D T_ab{vec_a};
    Twist2D twt_b{PI/6, 1.0, 2.0};

    // Answer
    Twist2D twt_ans{0.52359878, -0.04719755, 0.95280245};

    //Perform functions
    Twist2D res_twt = T_ab(twt_b);

    // Tests
    REQUIRE(res_twt.w == Approx ( twt_ans.w).margin(epsilon));
    REQUIRE(res_twt.vx == Approx( twt_ans.vx).margin(epsilon));
    REQUIRE(res_twt.vy == Approx( twt_ans.vy).margin(epsilon));
}

TEST_CASE("Transform2D::operator(Twist2D), Simple Transformation", "[Transform2D]") //Ryan King Shepard
{
    //Build objects
    Vector2D vec_a{2.0, -2.0};
    Transform2D T_ab{vec_a, PI/4};
    Twist2D twt_b{PI/6, 1.0, 2.0};

    // Answer
    Twist2D twt_ans{0.52359878, -1.75430433, 1.07412279};

    //Perform functions
    Twist2D res_twt = T_ab(twt_b);

    // Tests
    REQUIRE(res_twt.w == Approx ( twt_ans.w).margin(epsilon));
    REQUIRE(res_twt.vx == Approx( twt_ans.vx).margin(epsilon));
    REQUIRE(res_twt.vy == Approx( twt_ans.vy).margin(epsilon));
}

TEST_CASE("Pure Translation", "[integrate_twist]") { //Anna Garverick
    Twist2D trans; 
    trans.w = 0;
    trans.vx = 1;
    trans.vy = 2; 

    turtlelib::Transform2D pure_translation = turtlelib::integrate_twist(trans);
    turtlelib::Vector2D trans_out;
    trans_out.x = pure_translation.translation().x;
    trans_out.y = pure_translation.translation().y;

    REQUIRE(trans_out.x == 1);
    REQUIRE(trans_out.y == 2);
}

TEST_CASE("Pure Rotation", "[integrate_twist]") { //Anna Garverick
    Twist2D trans; 
    trans.w = .15;
    trans.vx = 0;
    trans.vy = 0; 

    turtlelib::Transform2D pure_rotation = turtlelib::integrate_twist(trans);
    double rot_out = pure_rotation.rotation();

    REQUIRE(rot_out == Approx(.15).margin(.01));
}

TEST_CASE("Simulatenous", "[integrate_twist]") { //Anna Garverick
    Twist2D trans; 
    trans.w = turtlelib::PI/2;
    trans.vx = 1;
    trans.vy = 1; 

    turtlelib::Transform2D sim = turtlelib::integrate_twist(trans);
    turtlelib::Vector2D trans_out = sim.translation();
    double rot_out = sim.rotation();

    REQUIRE(rot_out == Approx(turtlelib::PI/2).margin(.1));
    REQUIRE(trans_out.x == Approx(4/turtlelib::PI).margin(.1));
    REQUIRE(trans_out.y == Approx(0).margin(.1));
}