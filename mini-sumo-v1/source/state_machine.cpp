
#include <state_machine.h>

namespace SM
{

StateCondition::StateCondition(void) : condition([]{ return false; }), action([]{}){}
StateCondition::StateCondition(const std::function<bool()> test, const std::function<void()> onTrue) : condition(test), action(onTrue){}

bool StateCondition::resolve(void) const
{
    bool test = condition();
    if (test)
    {
        action();
        return true;
    }
    return false;
}

State::State(void) : conditions({}), onEnter([]{}), onExit([]{}){}
State::State(const std::vector<StateCondition> stateConditions, const std::function<void()> onEntry, const std::function<void()> onExit) : conditions(stateConditions), onEnter(onEntry), onExit(onExit){}
State::State(const StateCondition condition, const std::function<void()> onEntry, const std::function<void()> onExit) : conditions({condition}), onEnter(onEntry), onExit(onExit){}

bool State::resolve(void) const
{
    const size_t count = conditions.size();
    for (size_t i = 0; i < count; i++)
    {
        if (conditions[i].resolve())
        {
            return true;
        }
    }

    return false;
}

void State::enter(void) const
{
    onEnter();
}

void State::exit(void) const
{
    onExit();
}

StateMachine::StateMachine(const State* initialState) : current_state(nullptr), next_state(initialState){}

void StateMachine::next(const State* next)
{
    if (current_state != nullptr)
    {
        push(current_state);
    }
    next_state = next;
}

void StateMachine::previous(void)
{
    const State* p = pop();
    next_state = p;
}

void StateMachine::branch(const State* next)
{
    if (current_state != nullptr)
    {
        push(current_state, true);
    }
    next_state = next;
}

void StateMachine::jump(const State* next)
{
    if (current_state != nullptr)
    {
        push(current_state);
    }
    next_state = next;
}

void StateMachine::branch_return(int n)
{
    const State* p = nullptr;
    while (n--)
    {
        p = pop(true);
    }
    next_state = p;
}

void StateMachine::loop()
{
    if (current_state != nullptr && current_state == next_state)
    {
        bool resolved = current_state->resolve();
    }

    if (current_state != next_state)
    {
        if (current_state != nullptr)
        {
            current_state->exit();
        }

        if (next_state != nullptr)
        {
            next_state->enter();
        }
        
        current_state = next_state;
    }
}

void StateMachine::push(const State* state, bool branch)
{
    if (state == nullptr) return;
    struct StackEntry entry = { state , branch };
    state_stack.push(entry);
}

const State* StateMachine::pop(bool branch)
{
    const State* result = nullptr;

    if (!branch)
    {
        if (!state_stack.empty())
        {
            StackEntry& entry = state_stack.top();
            result = entry.p;
            if (state_stack.size() > 1)
                state_stack.pop();
        }
        else
        {
            result = nullptr;
        }
    }
    else
    {
        bool found = false;
        while (!found)
        {
            if (!state_stack.empty())
            {
                StackEntry& entry = state_stack.top();
                result = entry.p;

                if (state_stack.size() > 1)
                {
                    found = entry.branch;
                    state_stack.pop();
                }
                else
                {
                    found = true;
                }
            }
            else
            {
                result = nullptr;
                found = true;
            }
        }
    }

    return result;
}

};
