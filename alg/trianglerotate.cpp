#include "alg/trianglerotate.h"

#include <QtGlobal>
#include<QDebug>

TriangleRotateParticle::TriangleRotateParticle(const Node head,
                                               const int globalTailDir,
                                               const int orientation,
                                               AmoebotSystem& system,
                                               State state)
    : AmoebotParticle(head, globalTailDir, orientation, system),
      state(state),
      moveDir(-1),
      followDir(-1),
      possibleCenter(false),
      receivedCenterTokenFrom(-1) {
    //
}

void TriangleRotateParticle::activate() {
    switch(state) {
    case State::Idle:
    case State::Corner:
        findCenter();
        break;
    case State::Center:
    case State::CenterFound:
        bend();
        break;
    case State::Follow:
    case State::Head:
    case State::StaticEnd:
    case State::Finish:
        move();
    }
}

void TriangleRotateParticle::findCenter() {
    switch(state) {
    case State::Idle: { // an idle particle should check if it is a corner particle, then find the center
        std::vector<int> cornerLabels = isCorner();
        if (cornerLabels.size() == 2) {
            state = State::Corner;
            // send two counter tokens to the two sides
            int dir = cornerLabels[0] == 0 && cornerLabels[1] == 5 ? cornerLabels[1] : cornerLabels[0]; // pick the counter-clockwise first of the two
            // there should already be a neighbor at that position
            auto counterToken = std::make_shared<CounterToken>();
            counterToken->counter = 1; // starts at 0, already incremented once
            counterToken->passedFrom = getLabelPointsAtMe(dir);
            nbrAtLabel(dir).putToken(counterToken);

            auto markerToken = std::make_shared<MarkerToken>();
            markerToken->finished = true;
            markerToken->passedFrom = -1;
            this->putToken(markerToken);


        } else {
            // not a corner
            // if a counter was received, take it and pass it on, potentially creating a marker token travelling back
            if (hasToken<CounterToken>()) {
                auto counter = takeToken<CounterToken>();
                // pass on if possible.
                if (counter->counter == 0) {
                    // create marker token moving back
                    auto markerToken = std::make_shared<MarkerToken>();
                    markerToken->finished = false;
                    markerToken->passedFrom = getLabelPointsAtMe(counter->passedFrom);
                    nbrAtLabel(counter->passedFrom).putToken(markerToken);
                }

                counter -> counter = (counter->counter + 1) % 3;
                passTokenStraight(counter);
            }

            // pass on marker tokens
            if (hasToken<MarkerToken>()) {
                // pass on if possible
                auto marker = peekAtToken<MarkerToken>();
                if (!marker->finished) {
                    int newDir = (marker->passedFrom + 3) % 6;

                    // check if neighbor not already has a marker
                    if (hasNbrAtLabel(newDir)) {
                        int nbrLabelToMe = getLabelPointsAtMe(newDir);
                        bool safeToPassOn = true;

                        if (nbrAtLabel(newDir).hasToken<MarkerToken>()) {
                            auto neighborMarker = nbrAtLabel(newDir).peekAtToken<MarkerToken>();
                            if (neighborMarker->passedFrom == nbrLabelToMe || nbrAtLabel(newDir).state == State::Corner) { // If neighbor already has a marker token from me, do not send a new one.
                                safeToPassOn = false;
                                // instead check if that one is finished.
                                if (neighborMarker->finished) {
                                    // if that is the case, set this one to finished as well and put it back.
                                    marker->finished = true;
                                }
                            }
                        }
                        if (safeToPassOn) {
                            marker = takeToken<MarkerToken>();
                            passTokenStraight(marker);
                        }
                    }
                }
            }
            // check if a lastMarkerToken is present
            if (hasToken<LastMarkerToken>()) {
                auto lastToken = peekAtToken<LastMarkerToken>();
                if (lastToken->finished) {
                    // send a possible center token.
                    takeToken<LastMarkerToken>(); // remove the lastMarkerToken to make sure a center token is only send once.
                    int dir = (lastToken->passedFrom + 1) % 6;
                    auto centerToken = std::make_shared<CenterToken>();
                    centerToken->found = false;
                    centerToken->passedFrom = getLabelPointsAtMe(dir);
                    nbrAtLabel(dir).putToken(centerToken);
                }
            }


            // transmit center tokens
            if (hasToken<CenterToken>()) {
                auto centerToken = takeToken<CenterToken>();
                if (!centerToken->found) {
                    if (possibleCenter == false) {
                        possibleCenter = true;
                    } else { // already another center token passed. So this must be the center!
                        state = State::Center;
                        receivedCenterTokenFrom = centerToken->passedFrom;
                        // broadcast center found.
                        for (int i = 0; i < 6; i++) {
                            auto broadcast = std::make_shared<CenterToken>();
                            broadcast->found = true;
                            nbrAtLabel(i).putToken(broadcast);
                        }
                    }
                    passTokenStraight(centerToken);
                } else { // the center has been found, so broadcast it around
                    state = State::CenterFound;
                    for (int i = 0; i < 6; i++) {
                        if (hasNbrAtLabel(i)) {
                            if (nbrAtLabel(i).state != State::CenterFound) {
                                auto broadcast = std::make_shared<CenterToken>();
                                broadcast->found = true;
                                nbrAtLabel(i).putToken(broadcast);
                            }
                        }
                    }
                }
            }

        }
        break;
    }
    case State::Corner: {
        // a corner particle should send a finalMarkerToken back in case it receives a counterToken
        if (hasToken<CounterToken>()) {
            auto counter = takeToken<CounterToken>();
            Q_ASSERT(counter->counter == 0); // because this is a "perfect" triangle
            auto lastMarkerToken = std::make_shared<LastMarkerToken>();
            lastMarkerToken->finished = false;
            lastMarkerToken->passedFrom = getLabelPointsAtMe(counter->passedFrom);
            nbrAtLabel(counter->passedFrom).putToken(lastMarkerToken);
        }
        // if a centerfound token appears, change state to center found
        if (hasToken<CenterToken>()) {
            if (takeToken<CenterToken>()->found) {
                state = State::CenterFound;
            } else {
                // a non-found center token should not appear here as this particle can never be a possible center
            }
        }
        break;
    }
    default:
        printf("Find center called with invalid state: %s", stateString(state).toLocal8Bit().constData());
        throw 1;
    }
}

void TriangleRotateParticle::bend() {
    switch(state) {
    case State::Center:
        // send a bendtoken to each direction
        for (int offset = 0; offset < 6; offset+=2) {
            int dir = (receivedCenterTokenFrom + offset) % 6;
            auto staticBend = std::make_shared<BendPointToken>();
            staticBend->final = true;
            staticBend->passedFrom = getLabelPointsAtMe(dir);
            nbrAtLabel(dir).putToken(staticBend);

            dir = (dir + 1) % 6;
            auto nonStaticBend = std::make_shared<BendPointToken>();
            nonStaticBend->final = false;
            nonStaticBend->passedFrom = getLabelPointsAtMe(dir);
            nbrAtLabel(dir).putToken(nonStaticBend);
        }
        state = State::Finish;
        break;
    case State::CenterFound:
        // if a bend token was received, either finish or send out follow tokens to both rows
        if (hasToken<BendPointToken>()) {
            auto bendToken = takeToken<BendPointToken>();
            if (bendToken->final) {
                if (hasNbrAtLabel((bendToken->passedFrom + 3) % 6)) {
                    // not the last one
                    state = State::Finish;
                } else {
                    state = State::StaticEnd;
                    followDir = (bendToken->passedFrom + 4) % 6;
                }
            } else {
                // set following status and send follow tokens along the rows.
                state = State::Follow;
                followDir = (bendToken->passedFrom + 2) % 6;
                auto IFollowYou = std::make_shared<FollowToken>();
                IFollowYou->follow = false;
                if (hasNbrAtLabel(followDir)) {
                    IFollowYou->passedFrom = getLabelPointsAtMe(followDir);
                    nbrAtLabel(followDir).putToken(IFollowYou);
                } else {
                    // no neighbor in the follow direction, so this particle is head
                    state = State::Head;
                    moveDir = followDir;
                }

                int YouFollowMeDir = (followDir + 2 ) % 6;
                if (hasNbrAtLabel(YouFollowMeDir)) {
                    auto YouFollowMe = std::make_shared<FollowToken>();
                    YouFollowMe->follow = true;
                    YouFollowMe->passedFrom = getLabelPointsAtMe(YouFollowMeDir);
                    nbrAtLabel(YouFollowMeDir).putToken(YouFollowMe);
                }
            }
            passTokenStraight(bendToken);
        }
        // if a followToken was received, follow the row and set the status
        if (hasToken<FollowToken>()) {
            auto followToken = takeToken<FollowToken>();
            if (followToken->follow) {
                // meaning I should follow where it came from
                state = State::Follow;
                followDir = followToken->passedFrom;
            } else {
                // I should follow the next in line
                state = State::Follow;
                followDir = (followToken->passedFrom + 3) % 6;
                if (!hasNbrAtLabel(followDir)) {
                    // no neighbor to follow, I am head
                    moveDir = followDir;
                    state = State::Head;
                }
            }
            passTokenStraight(followToken);
        }

        break;
    default:
        printf("Bend called with invalid state: %s", stateString(state).toLocal8Bit().constData());
        throw 1;
    }
}

void TriangleRotateParticle::move() {
    switch(state) {
    case State::Follow:
        if (!hasNbrInState({State::CenterFound})) {
            if (isContracted() && hasTailAtLabel(followDir)) {
                    auto nbr = nbrAtLabel(followDir);
                    int nbrContractionDir = nbrDirToDir(nbr, (nbr.tailDir() + 3) % 6);
                    if (!canPush(followDir)) {
                        printf("Cannot push, nbr existing: %d, nbr expanded: %d, I'm contracted: %d\n", hasNbrAtLabel(followDir), !nbrAtLabel(followDir).isContracted(), isContracted());
                    }
                    push(followDir);
                    followDir = nbrContractionDir;
                    return;
            } else if (!isContracted() && !hasTailFollower() && !hasNbrInState({State::CenterFound})) { // only contract if this particle is the last one.
                contractTail();
            } else if (isContracted() && hasNbrAtLabel(followDir) && nbrAtLabel(followDir).state == State::Finish) { // if we follow a finished particle and are contracted, we are also finished.
                state = State::Finish;
            }
        }
        break;
    case State::Head:
        // move in the specified direction. There should be no particle in the way and it is not allowed to "overtake".
        if (isContracted() && hasNbrAtLabel((moveDir + 5) % 6)) { // +5 and not -1 to prevent a negative number after modulo
            expand(moveDir);
        }
        if (isContracted() && hasToken<FinishToken>()) {
            state = State::Finish;
        }
        break;
    case State::StaticEnd:
        // send a finish token
        if (hasNbrAtLabel(followDir) && nbrAtLabel(followDir).isContracted()) {
            auto finishToken = std::make_shared<FinishToken>();
            finishToken->passedFrom = getLabelPointsAtMe(followDir);
            nbrAtLabel(followDir).putToken(finishToken);
            state = State::Finish;
        }
        break;
    case State::Finish:
        // Keep on passing the finish tokens.
        if (hasToken<FinishToken>()) {
            int passDir = (peekAtToken<FinishToken>()->passedFrom + 3) % 6;
            if (hasNbrAtLabel(passDir) && nbrAtLabel(passDir).isContracted()) {
                passTokenStraight(takeToken<FinishToken>());
            }
        }
        break;
    default:
        printf("Move called with invalid state: %s", stateString(state).toLocal8Bit().constData());
        throw 1;
    }
}

bool TriangleRotateParticle::passTokenStraight(std::shared_ptr<PassableToken> passableToken) {
    int passedFrom = passableToken->passedFrom;
    int newDir = (passedFrom + 3) % 6;
    if (hasNbrAtLabel(newDir)) {
        passableToken->passedFrom = getLabelPointsAtMe(newDir);
        nbrAtLabel(newDir).putToken(passableToken);
        return true;
    } else {
        return false;
    }
}

TriangleRotateParticle& TriangleRotateParticle::nbrAtLabel(int label) const {
    return AmoebotParticle::nbrAtLabel<TriangleRotateParticle>(label);
}


std::vector<int> TriangleRotateParticle::isCorner() {
    std::vector<int> empty;
    if (!isContracted()) return empty;

    std::vector<int> neighborLabels;


    for (int labelOffset = 0; labelOffset < 6; labelOffset++) { // 6 is maximum since it is contracted
        const int label = (labelOffset) % 6;
        if (hasNbrAtLabel(label)) {
            neighborLabels.push_back(label);
        }
    }

    if (neighborLabels.size() != 2) { // if less or more than 2 neighbors, no point in going on
        return empty;
    }

    // check adjacency of the two labels
    if (neighborLabels[0] + 1 == neighborLabels[1] || // adjacent labels
            neighborLabels[0] == neighborLabels[1] - 5) { // adjacent modulo 6
        return neighborLabels;
    } else {
        return empty;
    }
}

bool TriangleRotateParticle::hasTailFollower() const {
  auto prop = [&](const TriangleRotateParticle& p) {
    return p.state == State::Follow &&
           pointsAtMyTail(p, p.dirToHeadLabel(p.followDir));
  };

  return labelOfFirstNbrWithProperty<TriangleRotateParticle>(prop) != -1;
}

int TriangleRotateParticle::labelOfFirstNbrInState(
    std::initializer_list<State> states, int startLabel = 0) const {
  auto prop = [&](const TriangleRotateParticle& p) {
    for (auto state : states) {
      if (p.state == state) {
        return true;
      }
    }
    return false;
  };

  return labelOfFirstNbrWithProperty<TriangleRotateParticle>(prop, startLabel);
}

bool TriangleRotateParticle::hasNbrInState(std::initializer_list<State> states)
    const {
  return labelOfFirstNbrInState(states) != -1;
}

int TriangleRotateParticle::getLabelPointsAtMe(int label) {
    auto nbr = nbrAtLabel(label);
    for (int nbrLabel = 0; nbrLabel < 6; nbrLabel++) {
        if (pointsAtMe(nbr, nbrLabel)) {
            return nbrLabel;
        }
    }
    // should never happen
    printf("This neighbor (%d, %d) has no label pointing at me (%d, %d) below 6. nbr expanded: %d?\n", nbr.head.x, nbr.head.y, this->head.x, this->head.y, !nbr.isContracted());
    throw 1;
}

int TriangleRotateParticle::headMarkColor() const {
    switch(state) {
    case State::Center: return 0x00ff00;
    case State::Corner:
    case State::Idle:
        if (hasToken<MarkerToken>()) {
            if (hasToken<LastMarkerToken>()) {
                return 0x000000;
            }
            if (peekAtToken<MarkerToken>()->finished) {
                return 0xff0000;
            } else {
                return 0xffff00;
            }
        } else if (possibleCenter) {
            return 0x00ff00;
        } else {
            return -1;
        }
    case State::CenterFound:
        return 0x00ffff;
    case State::Finish:
    case State::StaticEnd:
        return 0x000000;
    case State::Follow:
        return 0xff00ff;
    case State::Head:
        return 0xff0000;
    default:
        return -1;
    }
}

int TriangleRotateParticle::headMarkDir() const {
    switch(state) {
    case State::Idle:
        if (hasToken<MarkerToken>()) {
            return (peekAtToken<MarkerToken>()->passedFrom + 3) % 6;
        } else {
            return -1;
        }
    case State::StaticEnd:
    case State::Follow:
        if (followDir != -1) {
            return followDir;
        } else {
            return -1;
        }
    case State::Head:
        if (moveDir != -1) {
            return moveDir;
        } else {
            return -1;
        }
    case State::Corner:
    case State::Center:
    case State::Finish:
    case State::CenterFound:
    default:
        return -1;
    }
}

int TriangleRotateParticle::tailMarkColor() const {
    return headMarkColor();
}

QString TriangleRotateParticle::stateString(State s) const {
    switch(s) {
    case State::Center:
        return "Center";
    case State::Corner:
        return "Corner";
    case State::Finish:
        return "Finish";
    case State::CenterFound:
        return "CenterFound";
    case State::Idle:
        return "Idle";
    case State::StaticEnd:
        return "StaticEnd";
    case State::Follow:
        return "Follow";
    case State::Head:
        return "Head";
    default:
        printf("This particle has a state that has not been transcribed properly.");
        throw 1;
    }
}

QString TriangleRotateParticle::inspectionText() const {
    QString text;
    text += "Global Info:\n";
    text += "  head: (" + QString::number(head.x) + ", "
                        + QString::number(head.y) + ")\n";
    text += "  orientation: " + QString::number(orientation) + "\n";
    text += "  globalTailDir: " + QString::number(globalTailDir) + "\n\n";
    text += "Local Info:\n";
    text += "  State: " + stateString(state) + "\n";
    if (hasToken<MarkerToken>()) {
        text += "  Marker token: passedFrom: " + QString::number(peekAtToken<MarkerToken>()->passedFrom) + " finished: " + QString::number(peekAtToken<MarkerToken>()->finished) + "\n";
    }
    if (hasToken<CounterToken>()) {
        text += "  Counter token: passedFrom: " +QString::number(peekAtToken<CounterToken>()->passedFrom) + " counter: " + QString::number(peekAtToken<CounterToken>()->counter) + "\n";
    }
    if (hasToken<CenterToken>()) {
        text += "  Center token: passedFrom: " + QString::number(peekAtToken<CenterToken>()->passedFrom) + "\n";
    }
    if (hasToken<FinishToken>()) {
        text += "  Finish token: passedFrom: " + QString::number(peekAtToken<FinishToken>()->passedFrom) + "\n";
    }
    return text;
}


TriangleRotateSystem::TriangleRotateSystem(int sideLength, bool setCenter) {
    Q_ASSERT(sideLength % 3 == 1); // Should be a "perfect" triangle

    // Insert bottom left at (0, 0)
    int third = (sideLength - 1) / 3;
    for (int MaxXRow = sideLength; MaxXRow > 0; MaxXRow--) {
        for (int x = 0; x < MaxXRow; x++) {
            if (setCenter) {
                if (x == third && sideLength - MaxXRow == third) {
                    auto p = new TriangleRotateParticle(Node(x, sideLength - MaxXRow), -1, randDir(), *this, TriangleRotateParticle::State::Center);
                    if (p->orientation % 2 == 0) {
                        p->receivedCenterTokenFrom = 0;
                    } else {
                        p->receivedCenterTokenFrom = 1;
                    }
                    insert(p);
                } else {
                    auto p = new TriangleRotateParticle(Node(x, sideLength - MaxXRow), -1, randDir(), *this, TriangleRotateParticle::State::CenterFound);
                    insert(p);
                }
            } else {
                insert(new TriangleRotateParticle(Node(x, sideLength - MaxXRow), -1, randDir(), *this, TriangleRotateParticle::State::Idle));
            }
        }
    }
}

bool TriangleRotateSystem::hasTerminated() const {
    for (auto p : particles) {
        auto hp = dynamic_cast<TriangleRotateParticle*>(p);
        if (hp->state != TriangleRotateParticle::State::Finish &&
                hp->state != TriangleRotateParticle::State::Center) {
            return false;
        }
    }
    return true;
}
