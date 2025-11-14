
#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "pico/stdlib.h"
#include <stack>
#include <functional>
#include <vector>

namespace SM
{

class StateCondition
{
private:
    const std::function<bool()> condition;
    const std::function<void()> action;
public:
    StateCondition(void);
    StateCondition(const std::function<bool()> test, const std::function<void()> onTrue);
    bool resolve(void) const;
};

class State
{
private:
    const std::vector<StateCondition> conditions;
    const std::function<void()> onEnter;
    const std::function<void()> onExit;
public:
    State(void);
    State(const std::vector<StateCondition> stateConditions, const std::function<void()> onEntry = []() {}, const std::function<void()> onExit = []() {});
    State(const StateCondition conditions, const std::function<void()> onEntry = []() {}, const std::function<void()> onExit = []() {});
    bool resolve(void) const;
    void enter(void) const;
    void exit(void) const;
};

class StateMachine
{
private:
    struct StackEntry
    {
        const State* p;
        bool branch;
    };

    const State* current_state;
    const State* next_state;
    std::stack<StackEntry> state_stack;

    void push(const State* state, bool branch = false);
    const State* pop(bool branch = false);
public:
    StateMachine(const State* initialState = nullptr);
    void next(const State* next);
    void previous(void);
    void branch(const State* next);
    void jump(const State* next);
    void branch_return(int n = 1);
    void loop();
};

};



#endif // STATE_MACHINE_H
