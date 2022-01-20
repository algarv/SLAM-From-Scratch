# Turtlelib Library
A library for handling transformations in SE(2) and other turtlebot-related math.

# Components
- rigid2d - Handles 2D rigid body transformations
- frame_main - Perform some rigid body computations based on user input

# Conceptual Questions
1. We need to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality
        
        * Write the function independently in the header file
        * Include the function as a member of another class or structure
        * Create a class and with the functionality written into a constructor

   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.

        * F.2: A function should perform a single logical operation

        * C.1: Organize related data into structures (structs or classes)
        
        Writing a function independently is the simplest implementation for operations that will be used independently. Including the function as a member will allow interaction with similar functions and sharing private variables locally between the member functions. Creating a whole class around the functionality is the most complex option, but will be useful if there are secondary functions that could add more functionality (like how we added inv() and *= to Transform2D). 

   - Which of the methods would you implement and why?

        * In this case, the normalize function is not related to any of our classes or structs and only intended to be used to normalize vectors, so it is simplest to write it independently. 

2. What is the difference between a class and a struct in C++?
    * A class defaults to private elements that can only be accessed from within the class, while a struct  defaults to public elements that can  be accessible externally.

3. Why is Vector2D a struct and Transform2DClass (refer to at least 2 specific C++ core guidelines in your answer)?
    * C.1: Organize related data into structures (structs or classes)

    * C.2: Use class if the class has an invariant; use struct if the data members can vary independently


        A 2D vector has elements that we assign independently, while a transformation matrix has interdependent elements. We also want to access elements of the vector, but it is much more convenient to manipulate a transformation matrix as a chunk, rather than referencing individual elements. Therefor, Vector2D should be a struct while Transform2D should be a class.

4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?

    * C.46: By default, declare single-argument constructors explicit

        Single-argument constructors have an explicit argument to avoid an unintended implicit type conversion.

5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?

    * Con.1: By default, make objects immutable

    * Con.4: Use const to define objects with values that do not change after construction

        inv() should not modify the argument variable, so by default it is best to include the const argument and ensure the argument variable is not reassigned. operator*= actually intends to modify the lhs argument, so the const argument is not used. 

# Sample Run of frame_main
```
Enter transform T_{a,b}:
deg: 90 x: 0 y: 1
Enter transform T_{b,c}:
deg: 90 x: 1 y: 0
T_{a,b}: deg: 90 x: 0 y: 1
T_{b,a}: deg: -90 x: -1 y: -6.12323e-17
T_{b,c}: deg: 90 x: 1 y: 0
T_{c,b}: deg: -90 x: -6.12323e-17 y: 1
T_{a,c}: deg: 180 x: 6.12323e-17 y: 2
T_{c,a}: deg: -180 x: -1.83697e-16 y: 2
Enter vector v_b:
1 1
vb_hat: [0.707107 0.707107]
v_a: [-1 2]
v_b: [1 1]
v_c: [1 1.11022e-16]
Enter twist V_b:
1 1 1
Va: [1 0 1]
Vb: [1 1 1]
Vc: [1 2 -1]
```