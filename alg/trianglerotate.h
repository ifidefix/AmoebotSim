// Defines the particle system and composing particles for the triangle rotation
//
// the sidelength l of the triangle should be 3k+1

#ifndef TRIANGLEROTATE_H
#define TRIANGLEROTATE_H

#include <set>
#include <vector>

#include <QString>

#include "core/amoebotparticle.h"
#include "core/amoebotsystem.h"

class TriangleRotateParticle : public AmoebotParticle {
public:
    enum class State {
        Idle,
        Center,
        StaticEnd,
        Finish,
        Corner,
        CenterFound,
        Follow,
        Head
    };
    // Turns the state into a string
    QString stateString(State s) const;

    // Constructs a new particle with a node position for its head, a global
    // compass direction from its head to its tail (-1 if contracted), an offset
    // for its local compass, a system which it belongs to and an initial state
    TriangleRotateParticle(const Node head, const int globalTailDir,
                           const int orientation, AmoebotSystem& system,
                           State state);

    // Gets a reference to the neighboring particle incident to the specified port
    // label. Crashes if no such particle exists at this label; consider using
    // hasNbrAtLabel() first if unsure.
    TriangleRotateParticle& nbrAtLabel(int label) const;

    // Executes one particle activation
    virtual void activate();

    // Functions for altering a particle's cosmetic appearance; headMarkColor
    // (respectively, tailMarkColor) returns the color to be used for the ring
    // drawn around the head (respecitvely, tail) node. Tail color is not shown
    // when the particle is contracted. headMarkDir returns the label of the port
    // on which the black head marker is drawn.
    virtual int headMarkColor() const;
    virtual int headMarkDir() const;
    virtual int tailMarkColor() const;

    // Returns the string to be displayed when this particle is inspected; used
    // to snapshot the current values of this particle's memory at runtime.
    virtual QString inspectionText() const;

    // Checks if this particle has exactly 2 neighboring particles adjacent to each other
    std::vector<int> isCorner();

    // Take the neighbor at the given label and calculate the label that points at this particle
    // crashes if there is no particle at the given label, consider using hasNbrAtLabel()
    int getLabelPointsAtMe(int label);

    // Find out if there is a particle following this particle
    bool hasTailFollower() const;

    // Helper function to find the first neighbor in a specific state
    int labelOfFirstNbrInState(std::initializer_list<State> states, int startLabel) const;

    // Helper function to check if there is a neighbor in a specific state.
    bool hasNbrInState(std::initializer_list<State> states) const;

    // Part of the algorithm. The purpose is to find the center. Activates in states Idle, or Corner
    void findCenter();

    // The other part of the algorithm. Setting all directions up for the moving part
    void bend();

    // Once the rows and follower relations are established, actually move.
    void move();

protected:
    State state;
    int moveDir;
    int followDir;
    bool possibleCenter;
    int receivedCenterTokenFrom; // the direction from which this token has received a center token. Used by the center to distribute the bending tokens.

    // Token types
    // A token to be passed straight on.
    struct PassableToken : public Token { int passedFrom = -1; };
    // counter token counts from corners of triangles modulo 3
    struct CounterToken : public PassableToken { int counter = 0; };
    // marker tokens to count to 1/3th of the sidelength
    struct MarkerToken : public PassableToken { bool finished = false; };
    // Last marker token is the last marker token that is send
    struct LastMarkerToken : public MarkerToken {};
    // Token used to indicate a possible center. Also used to broadcast that a center has been found.
    struct CenterToken : public PassableToken { bool found = false; };

    // Token to indicate a bendpoint in the structure. Final indicates if it is one of the static axis (true) or moving axis (false).
    struct BendPointToken : public PassableToken { bool final = true; };
    // Token send per row from the bendpoints. If follow == true, it means the receiving particle should follow, if it is false, the receiving particle should lead.
    struct FollowToken : public PassableToken { bool follow = false; };
    // Token send to indicate that a particle is on its final position.
    struct FinishToken : public PassableToken {};


    // Pass a token straight on. Returns true if it could be passed. False if there is no neighbor to pass it on to.
    bool passTokenStraight(std::shared_ptr<PassableToken> passableToken);


private:
    friend class TriangleRotateSystem;
};

class TriangleRotateSystem : public AmoebotSystem {
public:
    // Constructs a triangle of TriangleRotateParticles with an optionally specified sidelength l
    TriangleRotateSystem(int sideLength = 7, bool setCenter = false);

    // Checks whether or not the system's run of the TriangleRotate algorithym has terminated (all particles in state Finish).
    bool hasTerminated() const override;
};

#endif // TRIANGLEROTATE_H
