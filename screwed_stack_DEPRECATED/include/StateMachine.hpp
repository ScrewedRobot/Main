// StateMachine.hpp

#pragma once

#include <cstddef>


class StateMachine {
public:
    enum class State {
        INIT,
        STANDBY,
        AQUIRE,
        APPROACH,
        EXECUTE
    };

    StateMachine()
        : state_(State::INIT),
          prev_state_(State::INIT)
    {}

    virtual ~StateMachine() = default;

    // Check for state transitions
    void runChecks() {
        switch (state_) {
            case State::INIT:
                if (checkStandby()) {
                    state_ = State::STANDBY;
                    onEnterStandby();
                }
                break;

            case State::STANDBY:
                if (checkAquire()) {
                    state_ = State::AQUIRE;
                    onEnterAquire();
                } 
                else if (checkApproach()) {
                    state_ = State::APPROACH;
                    onEnterApproach();
                }
                break;

            case State::AQUIRE:
                if (checkStandby()) {
                    state_ = State::STANDBY;
                    onEnterStandby();
                }
                break;

            case State::APPROACH:
                if (checkStandby()) {
                    state_ = State::STANDBY;
                    onEnterStandby();
                }
                else if (checkExecute()) {
                    state_ = State::EXECUTE;
                    onEnterExecute();
                }
                break;

            case State::EXECUTE:
                if (checkApproach()) {
                    state_ = State::APPROACH;
                    onEnterApproach();
                }
                break;
        }

    }

    State currentState() const { return state_; }
    State previousState() const { return prev_state_; }
    
    
protected:
    State state_;
    State prev_state_;

    // State transition checks
    virtual bool checkStandby() = 0;
    virtual bool checkAquire() = 0;
    virtual bool checkApproach() = 0;
    virtual bool checkExecute() = 0;
    
    // Entry actions
    virtual void onEnterInit() = 0;
    virtual void onEnterStandby() = 0;
    virtual void onEnterAquire() = 0;
    virtual void onEnterApproach() = 0;
    virtual void onEnterExecute() = 0;


};



