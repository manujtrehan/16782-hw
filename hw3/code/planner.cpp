#include <iostream>
#include <fstream>
// #include <boost/functional/hash.hpp>
#include <regex>
#include <unordered_set>
#include <set>
#include <list>
#include <unordered_map>
#include <algorithm>
#include <stdexcept>

#include <limits.h>
#include <queue>
#include <memory>

#define SYMBOLS 0
#define INITIAL 1
#define GOAL 2
#define ACTIONS 3
#define ACTION_DEFINITION 4
#define ACTION_PRECONDITION 5
#define ACTION_EFFECT 6

class GroundedCondition;
class Condition;
class GroundedAction;
class Action;
class Env;

using namespace std;

bool print_status = true;

class GroundedCondition
{
private:
    string predicate;
    list<string> arg_values;
    bool truth = true;

public:
    GroundedCondition(string predicate, list<string> arg_values, bool truth = true)
    {
        this->predicate = predicate;
        this->truth = truth;  // fixed
        for (string l : arg_values)
        {
            this->arg_values.push_back(l);
        }
    }

    GroundedCondition(const GroundedCondition& gc)
    {
        this->predicate = gc.predicate;
        this->truth = gc.truth;  // fixed
        for (string l : gc.arg_values)
        {
            this->arg_values.push_back(l);
        }
    }

    string get_predicate() const
    {
        return this->predicate;
    }
    list<string> get_arg_values() const
    {
        return this->arg_values;
    }

    bool get_truth() const
    {
        return this->truth;
    }

    friend ostream& operator<<(ostream& os, const GroundedCondition& pred)
    {
        os << pred.toString() << " ";
        return os;
    }

    bool operator==(const GroundedCondition& rhs) const
    {
        if (this->predicate != rhs.predicate || this->arg_values.size() != rhs.arg_values.size())
            return false;

        auto lhs_it = this->arg_values.begin();
        auto rhs_it = rhs.arg_values.begin();

        while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }

        if (this->truth != rhs.get_truth()) // fixed
            return false;

        return true;
    }

    string toString() const
    {
        string temp = "";
        // if (!this->truth) // added to ensure hash is unique for true/false
        //     temp += "!";
        temp += this->predicate;
        temp += "(";
        for (string l : this->arg_values)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct GroundedConditionComparator
{
    bool operator()(const GroundedCondition& lhs, const GroundedCondition& rhs) const
    {
        return lhs == rhs;
    }
};

struct GroundedConditionHasher
{
    size_t operator()(const GroundedCondition& gcond) const
    {
        return hash<string>{}(gcond.toString());
    }
};

class Condition
{
private:
    string predicate;
    list<string> args;
    bool truth;

public:
    Condition(string pred, list<string> args, bool truth)
    {
        this->predicate = pred;
        this->truth = truth;
        for (string ar : args)
        {
            this->args.push_back(ar);
        }
    }

    string get_predicate() const
    {
        return this->predicate;
    }

    list<string> get_args() const
    {
        return this->args;
    }

    bool get_truth() const
    {
        return this->truth;
    }

    friend ostream& operator<<(ostream& os, const Condition& cond)
    {
        os << cond.toString() << " ";
        return os;
    }

    bool operator==(const Condition& rhs) const // fixed
    {

        if (this->predicate != rhs.predicate || this->args.size() != rhs.args.size())
            return false;

        auto lhs_it = this->args.begin();
        auto rhs_it = rhs.args.begin();

        while (lhs_it != this->args.end() && rhs_it != rhs.args.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }

        if (this->truth != rhs.get_truth())
            return false;

        return true;
    }

    string toString() const
    {
        string temp = "";
        if (!this->truth)
            temp += "!";
        temp += this->predicate;
        temp += "(";
        for (string l : this->args)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct ConditionComparator
{
    bool operator()(const Condition& lhs, const Condition& rhs) const
    {
        return lhs == rhs;
    }
};

struct ConditionHasher
{
    size_t operator()(const Condition& cond) const
    {
        return hash<string>{}(cond.toString());
    }
};

class Action
{
private:
    string name;
    list<string> args;
    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects;

public:
    Action(string name, list<string> args,
        unordered_set<Condition, ConditionHasher, ConditionComparator>& preconditions,
        unordered_set<Condition, ConditionHasher, ConditionComparator>& effects)
    {
        this->name = name;
        for (string l : args)
        {
            this->args.push_back(l);
        }
        for (Condition pc : preconditions)
        {
            this->preconditions.insert(pc);
        }
        for (Condition pc : effects)
        {
            this->effects.insert(pc);
        }
    }
    string get_name() const
    {
        return this->name;
    }
    list<string> get_args() const
    {
        return this->args;
    }
    unordered_set<Condition, ConditionHasher, ConditionComparator> get_preconditions() const
    {
        return this->preconditions;
    }
    unordered_set<Condition, ConditionHasher, ConditionComparator> get_effects() const
    {
        return this->effects;
    }

    bool operator==(const Action& rhs) const
    {
        if (this->get_name() != rhs.get_name() || this->get_args().size() != rhs.get_args().size())
            return false;

        return true;
    }

    friend ostream& operator<<(ostream& os, const Action& ac)
    {
        os << ac.toString() << endl;
        os << "Precondition: ";
        for (Condition precond : ac.get_preconditions())
            os << precond;
        os << endl;
        os << "Effect: ";
        for (Condition effect : ac.get_effects())
            os << effect;
        os << endl;
        return os;
    }

    string toString() const
    {
        string temp = "";
        temp += this->get_name();
        temp += "(";
        for (string l : this->get_args())
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct ActionComparator
{
    bool operator()(const Action& lhs, const Action& rhs) const
    {
        return lhs == rhs;
    }
};

struct ActionHasher
{
    size_t operator()(const Action& ac) const
    {
        return hash<string>{}(ac.get_name());
    }
};

class Env
{
private:
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> initial_conditions;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> goal_conditions;
    unordered_set<Action, ActionHasher, ActionComparator> actions;
    unordered_set<string> symbols;

public:
    void remove_initial_condition(GroundedCondition gc)
    {
        this->initial_conditions.erase(gc);
    }
    void add_initial_condition(GroundedCondition gc)
    {
        this->initial_conditions.insert(gc);
    }
    void add_goal_condition(GroundedCondition gc)
    {
        this->goal_conditions.insert(gc);
    }
    void remove_goal_condition(GroundedCondition gc)
    {
        this->goal_conditions.erase(gc);
    }
    void add_symbol(string symbol)
    {
        symbols.insert(symbol);
    }
    void add_symbols(list<string> symbols)
    {
        for (string l : symbols)
            this->symbols.insert(l);
    }
    void add_action(Action action)
    {
        this->actions.insert(action);
    }

    Action get_action(string name)
    {
        for (Action a : this->actions)
        {
            if (a.get_name() == name)
                return a;
        }
        throw runtime_error("Action " + name + " not found!");
    }
    unordered_set<string> get_symbols() const
    {
        return this->symbols;
    }

    unordered_set<Action, ActionHasher, ActionComparator> get_all_actions()
    {
        return this->actions;
    }

    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_initial_condition() const
    {
        return this->initial_conditions;
    }

    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_goal_condition() const
    {
        return this->goal_conditions;
    }

    friend ostream& operator<<(ostream& os, const Env& w)
    {
        os << "***** Environment *****" << endl << endl;
        os << "Symbols: ";
        for (string s : w.get_symbols())
            os << s + ",";
        os << endl;
        os << "Initial conditions: ";
        for (GroundedCondition s : w.initial_conditions)
            os << s;
        os << endl;
        os << "Goal conditions: ";
        for (GroundedCondition g : w.goal_conditions)
            os << g;
        os << endl;
        os << "Actions:" << endl;
        for (Action g : w.actions)
            os << g << endl;
        cout << "***** Environment Created! *****" << endl;
        return os;
    }
};

class GroundedAction
{
private:
    string name;
    list<string> arg_values;

public:
    GroundedAction() // default constructor since it is a member in Node, and needs an empty constructor
    {
        this->name = "";
        this->arg_values = list<string>{};
    }

    GroundedAction(string name, list<string> arg_values)
    {
        this->name = name;
        for (string ar : arg_values)
        {
            this->arg_values.push_back(ar);
        }
    }

    string get_name() const
    {
        return this->name;
    }

    list<string> get_arg_values() const
    {
        return this->arg_values;
    }

    bool operator==(const GroundedAction& rhs) const
    {
        if (this->name != rhs.name || this->arg_values.size() != rhs.arg_values.size())
            return false;

        auto lhs_it = this->arg_values.begin();
        auto rhs_it = rhs.arg_values.begin();

        while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }
        return true;
    }

    friend ostream& operator<<(ostream& os, const GroundedAction& gac)
    {
        os << gac.toString() << " ";
        return os;
    }

    string toString() const
    {
        string temp = "";
        temp += this->name;
        temp += "(";
        for (string l : this->arg_values)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

list<string> parse_symbols(string symbols_str)
{
    list<string> symbols;
    size_t pos = 0;
    string delimiter = ",";
    while ((pos = symbols_str.find(delimiter)) != string::npos)
    {
        string symbol = symbols_str.substr(0, pos);
        symbols_str.erase(0, pos + delimiter.length());
        symbols.push_back(symbol);
    }
    symbols.push_back(symbols_str);
    return symbols;
}

Env* create_env(char* filename)
{
    ifstream input_file(filename);
    Env* env = new Env();
    regex symbolStateRegex("symbols:", regex::icase);
    regex symbolRegex("([a-zA-Z0-9_, ]+) *");
    regex initialConditionRegex("initialconditions:(.*)", regex::icase);
    regex conditionRegex("(!?[A-Z][a-zA-Z_]*) *\\( *([a-zA-Z0-9_, ]+) *\\)");
    regex goalConditionRegex("goalconditions:(.*)", regex::icase);
    regex actionRegex("actions:", regex::icase);
    regex precondRegex("preconditions:(.*)", regex::icase);
    regex effectRegex("effects:(.*)", regex::icase);
    int parser = SYMBOLS;

    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects;
    string action_name;
    string action_args;

    string line;
    if (input_file.is_open())
    {
        while (getline(input_file, line))
        {
            string::iterator end_pos = remove(line.begin(), line.end(), ' ');
            line.erase(end_pos, line.end());

            if (line == "")
                continue;

            if (parser == SYMBOLS)
            {
                smatch results;
                if (regex_search(line, results, symbolStateRegex))
                {
                    line = line.substr(8);
                    sregex_token_iterator iter(line.begin(), line.end(), symbolRegex, 0);
                    sregex_token_iterator end;

                    env->add_symbols(parse_symbols(iter->str()));  // fixed

                    parser = INITIAL;
                }
                else
                {
                    cout << "Symbols are not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == INITIAL)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, initialConditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_initial_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_initial_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = GOAL;
                }
                else
                {
                    cout << "Initial conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == GOAL)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, goalConditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_goal_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_goal_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = ACTIONS;
                }
                else
                {
                    cout << "Goal conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTIONS)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, actionRegex))
                {
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Actions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_DEFINITION)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, conditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;
                    // name
                    action_name = iter->str();
                    iter++;
                    // args
                    action_args = iter->str();
                    iter++;

                    parser = ACTION_PRECONDITION;
                }
                else
                {
                    cout << "Action not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_PRECONDITION)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, precondRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition precond(predicate, parse_symbols(args), truth);
                        preconditions.insert(precond);
                    }

                    parser = ACTION_EFFECT;
                }
                else
                {
                    cout << "Precondition not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_EFFECT)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, effectRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition effect(predicate, parse_symbols(args), truth);
                        effects.insert(effect);
                    }

                    env->add_action(
                        Action(action_name, parse_symbols(action_args), preconditions, effects));

                    preconditions.clear();
                    effects.clear();
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Effects not specified correctly." << endl;
                    throw;
                }
            }
        }
        input_file.close();
    }

    else
        cout << "Unable to open file";

    return env;
}




/** Start **/



struct CustomGC
{
    bool operator()(const GroundedCondition& lhs, const GroundedCondition& rhs) const
    {
        return lhs.toString() < rhs.toString();
    }
};

struct Node
{
    // an alphabetically ordered set of conditions defining the state. order needed for hashing
    set<GroundedCondition, CustomGC> conditions;
    // g value of the state
    int gVal;
    int hVal;
    int fVal;
    shared_ptr<Node> parent; // parent node
    GroundedAction parent_action; // what action does the parent perform to reach current state

    // define constructors
    Node() : gVal(INT_MAX), fVal(INT_MAX), parent(nullptr) {}
    Node(const int g) : gVal(g), parent(nullptr) {}
    Node(const set<GroundedCondition, CustomGC>& c) : conditions(c), gVal(INT_MAX), fVal(INT_MAX), parent(nullptr) {}
    Node(const set<GroundedCondition, CustomGC>& c, const int h) : conditions(c), gVal(INT_MAX), fVal(INT_MAX), hVal(h), parent(nullptr) {}

    bool operator==(const Node& rhs) const
    {
        if (this->conditions.size() != rhs.conditions.size())
            return false;

        return (this->conditions == rhs.conditions);
    }
};

struct NodeHasher
{
    size_t operator()(const set<GroundedCondition, CustomGC>& gc_set) const
    {
        string hashStr;
        for(auto cond : gc_set)
        {
            hashStr += cond.toString();
        }
        return hash<string>{}(hashStr);
    }
};

struct NodeComparator
{
    bool operator()(const set<GroundedCondition, CustomGC>& lhs, const set<GroundedCondition, CustomGC>& rhs) const
    {
        return lhs == rhs;
    }
};

struct ActionMapHasher
{
    size_t operator()(const set<GroundedCondition, CustomGC>& gc_set) const
    {
        string hashStr;
        for(auto gc : gc_set)
        {
            hashStr += gc.toString();
        }
        return hash<string>{}(hashStr);
    }
};

struct ActionMapComparator
{
    bool operator()(const set<GroundedCondition, CustomGC>& lhs, const set<GroundedCondition, CustomGC>& rhs) const
    {
        return lhs == rhs;
    }
};

static auto compare = [](const shared_ptr<Node>& n1, const shared_ptr<Node>& n2)
{
    // return n1->gVal > n2->gVal;
    return n1->fVal > n2->fVal;
};

struct MultiDimMap
{
    unordered_map<GroundedCondition, shared_ptr<MultiDimMap>, GroundedConditionHasher, GroundedConditionComparator> multimap;
    vector<pair<unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>, GroundedAction> > value;
    bool end;

    MultiDimMap() : end(false) {}
};

// globals
priority_queue<shared_ptr<Node>, vector<shared_ptr<Node> >, decltype(compare)> open_queue(compare);
unordered_map<set<GroundedCondition, CustomGC>, shared_ptr<Node>, NodeHasher, NodeComparator> nodes;
unordered_set<set<GroundedCondition, CustomGC>, NodeHasher, NodeComparator> closed;

// action_map maps states/preconditions to possible pair of (effects, grounded action)
// ordered set because Node has an ordered set. can use its member directly as a key
MultiDimMap action_map;

void generateCombos(const vector<string>& sym, int start, int k, vector<string>& curr, vector<vector<string> >& combos)
{
    if(k == 0)
    {
        combos.push_back(curr);
        return;
    }
    for(int i = start; i <= (sym.size() - k); ++i)
    {
        curr.push_back(sym[i]);
        generateCombos(sym, i + 1, k - 1, curr, combos);
        curr.pop_back();
    }
    return;
}

void buildActionMap(
        unordered_set<Action, ActionHasher, ActionComparator>& actions,
        unordered_set<string>& symbols)
{
    vector<string> sym;
    for(string s : symbols)
    {
        sym.push_back(s);
    }

    sort(sym.begin(), sym.end()); // sort symbols alphabetically

    vector<string> curr;
    vector<vector<string> > combos; // vector to store generated combos. need to permute over each element
    set<GroundedCondition, CustomGC> preconds; // set of preconditions to add to the action map
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> effects; // set of effects to match the preconditions and add to action map
    unordered_map<string, string> sym_map; // map to match symbols between grounded and non-grounded stuff

    // loop over all possible actions in env
    for(auto& action : actions)
    {
        combos.clear(); // clear old combinations
        int num_args = action.get_args().size(); // number of args in the action for combinations

        // cout << "Name of action: " << action.get_name() << endl;
        // for(auto q : action.get_preconditions())
        // {
        //     cout << q << endl;
        // }

        // generate all possible combinations of size 'num_args' from sym. stored in combos. then permute each element
        generateCombos(sym, 0, num_args, curr, combos);
        // cout << "Combo list size: " << combos.size() << endl;

        // permute each element in combos
        for(vector<string> symbol_seq : combos)
        {
            // cout << "Next Combo" << endl;
            do
            {
                // for(auto p : symbol_seq)
                // {
                //     cout << p << " ";
                // }
                // cout << endl;

                sym_map.clear(); // clear the symbol map
                int ind = 0;
                list<string> ga_syms; // list of actual symbols for grounded action
                for(string action_sym : action.get_args()) // map action symbols (x, y, b) to actual symbols (A, B, Table) in order
                {
                    sym_map[action_sym] = symbol_seq[ind];
                    ga_syms.push_back(symbol_seq[ind]);
                    ++ind;
                }

                // generate and add precondition for this sequence of symbols for action_map
                preconds.clear();
                for(auto pc : action.get_preconditions())
                {
                    // pc is of type Condition - convert to a GroundedCondition
                    list<string> actual_args;
                    for(string pc_arg : pc.get_args())
                    {
                        if(sym_map.find(pc_arg) != sym_map.end()) // if arg not in sym_map, use default arg
                            actual_args.push_back(sym_map[pc_arg]);
                        else actual_args.push_back(pc_arg);
                    }
                    GroundedCondition gc_precond(pc.get_predicate(), actual_args, pc.get_truth());
                    preconds.insert(gc_precond);
                }

                // generate and add effects for this precondition for action_map
                effects.clear();
                for(auto eff : action.get_effects())
                {
                    // eff is of type Condition - convert it to a GroundedCondition
                    list<string> actual_args;
                    for(string eff_arg : eff.get_args())
                    {
                        if(sym_map.find(eff_arg) != sym_map.end()) // if arg not in sym_map, use default arg
                            actual_args.push_back(sym_map[eff_arg]);
                        else actual_args.push_back(eff_arg);
                    }
                    GroundedCondition gc_eff(eff.get_predicate(), actual_args, eff.get_truth());
                    effects.insert(gc_eff);
                }

                // create the specific grounded action for action_map
                GroundedAction ga(action.get_name(), ga_syms);

                // cout << ga << endl;
                // cout << "Preconds" << endl;
                // for(auto m : preconds)
                // {
                //     cout << m << endl;
                // }
                // cout << "Effects" << endl;
                // for(auto m : effects)
                // {
                //     cout << m << endl;
                // }
                // cout << "----" << endl;

                // store precondition, effect and grounded action in multi layered action_map
                MultiDimMap* multimap_ptr = &action_map; // no need to delete pointer since memory not on heap. normal ptr since not dynamically alloc
                for(auto it = preconds.begin(); it != preconds.end(); ++it)
                {
                    if(multimap_ptr->multimap.find(*it) == multimap_ptr->multimap.end())
                        multimap_ptr->multimap[*it] = make_shared<MultiDimMap>();
                    // cout << *it << endl;
                    if(next(it) == preconds.end()) // reached end of precondition, store (effect, action) pair
                    {
                        multimap_ptr = multimap_ptr->multimap[*it].get(); // move to child node and store there
                        multimap_ptr->end = true;
                        multimap_ptr->value.push_back({effects, ga});
                        break;
                    }
                    else multimap_ptr = multimap_ptr->multimap[*it].get();
                }
                // action_map[preconds].push_back({effects, ga});

            } while (next_permutation(symbol_seq.begin(), symbol_seq.end()));   
        }
    }
}

tuple<bool, MultiDimMap*> findInActionMap(
                            const set<GroundedCondition, CustomGC>& state,
                            MultiDimMap* multimap_ptr,
                            // MultiDimMap* parent_ptr,
                            vector<MultiDimMap*>& output,
                            int n)
{
    // MultiDimMap* multimap_ptr = &action_map;
    // MultiDimMap* parent_ptr = nullptr;
    auto cond = state.begin();
    if(n > 0) advance(cond, n);

    for(auto end = state.end(); cond != end; ++cond)
    {
        if(multimap_ptr->multimap.find(*cond) != multimap_ptr->multimap.end())
        {
            // cout << "Debug: " << *cond << endl;
            MultiDimMap* next_ptr = multimap_ptr->multimap[*cond].get();
            // parent_ptr = multimap_ptr;
            findInActionMap(state, next_ptr, output, n + 1);
            // findInActionMap(state, next_ptr, parent_ptr, output, n + 1);
            // multimap_ptr = next_ptr;
        }
    }
    if(multimap_ptr->value.empty())
    {
        return make_tuple(false, nullptr);
    }
    else
    {
        output.push_back(multimap_ptr);
        // cout << "Returned" << endl;
        return make_tuple(true, multimap_ptr);
    }
}

void addEffectToState(
        set<GroundedCondition, CustomGC>& state,
        unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>& effects)
{
    for(auto effect : effects)
    {
        if(!effect.get_truth())
        {
            state.erase(effect);
        }
        else
        {
            state.insert(effect);
        }
    }
}

bool isGoal(
        set<GroundedCondition, CustomGC>& currState,
        set<GroundedCondition, CustomGC>& goal)
{
    for(auto gc : goal)
    {
        if(currState.find(gc) == currState.end()) // current state doesn't contain goal condition
            return false;
    }
    return true;
}

int computeHeuristic(
        set<GroundedCondition, CustomGC>& goal,
        set<GroundedCondition, CustomGC>& child_conditions)
{
    int h = 0;
    for(auto cond : goal)
    {
        if(child_conditions.find(cond) == child_conditions.end())
            ++h;
    }
    return h;
}

list<GroundedAction> computePath(set<GroundedCondition, CustomGC>& goal)
{
	int cost = 1;
	while(!open_queue.empty())
	{
        // cout << "Open queue size: " << open_queue.size() << endl;
		shared_ptr<Node> s = open_queue.top();
		open_queue.pop();
        // cout << "Current state:" << endl;
        // for(auto i : s->conditions)
        // {
        //     cout << i << endl;
        // }

		if(closed.find(s->conditions) != closed.end()) continue; // skip if already visited
		closed.insert(s->conditions); // add to closed list

		if(isGoal(s->conditions, goal)) // check if goal
		{
			// goal found. backtrack
			list<GroundedAction> plan;
            while(s)
            {
                plan.push_front(s->parent_action);
                s = s->parent;
            }
            plan.pop_front(); // remove starting empty action
            return plan;
		}

		unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> effect;
		GroundedAction grounded_action;
        // bool found;
        // MultiDimMap* solution_ptr;
        vector<MultiDimMap*> output;
        findInActionMap(s->conditions, &action_map, output, 0); // find possible actions based on current state
        // tie(found, solution_ptr) = findInActionMap(s->conditions, &action_map, nullptr, output, 0); // find possible actions based on current state
		// cout << output.size() << endl;
        for(MultiDimMap* solution_ptr : output)
        {
            for(auto effect_action_pair : solution_ptr->value)
            {
                tie(effect, grounded_action) = effect_action_pair;
                // cout << "Next action: " << grounded_action << endl;
                set<GroundedCondition, CustomGC> child_conditions = s->conditions; // copy parent conditions to child
                // add effects to child conditions
                addEffectToState(child_conditions, effect);

                if(closed.find(child_conditions) == closed.end())
                {
                    if(nodes.find(child_conditions) == nodes.end())
                    {
                        int h = computeHeuristic(goal, child_conditions);
                        shared_ptr<Node> newNode = make_shared<Node>(child_conditions, h);
                        nodes[child_conditions] = newNode;
                    }
                    if(nodes[child_conditions]->gVal > (s->gVal + cost))
                    {
                        nodes[child_conditions]->gVal = s->gVal + cost;
                        nodes[child_conditions]->fVal = nodes[child_conditions]->gVal + nodes[child_conditions]->hVal;
                        nodes[child_conditions]->parent = s;
                        nodes[child_conditions]->parent_action = grounded_action;
                        open_queue.push(nodes[child_conditions]);
                    }
                }
            }
        }
        // break;
        // cout << endl;
	}

	return list<GroundedAction>{};
}


set<GroundedCondition, CustomGC> unorderedToOrderedGC(
                                    const unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>& uos)
{
    set<GroundedCondition, CustomGC> out;
    for(auto val : uos)
    {
        out.insert(val);
    }
    return out;
}

list<GroundedAction> planner(Env* env)
{
    // this is where you insert your planner

    // get and order init and goal conditions to compare
    set<GroundedCondition, CustomGC> init, goal;
    init = unorderedToOrderedGC(env->get_initial_condition());
    goal = unorderedToOrderedGC(env->get_goal_condition());

    unordered_set<Action, ActionHasher, ActionComparator> all_actions = env->get_all_actions();
    unordered_set<string> all_symbols = env->get_symbols();
    buildActionMap(all_actions, all_symbols);

    // for(auto i : init)
    // {
    //     cout << i << endl;
    // }
    // cout << endl;
    // cout << "Start:" << endl;

    int h_start = computeHeuristic(goal, init);

    shared_ptr<Node> start = make_shared<Node>(init, h_start);
    start->gVal = 0;
    start->fVal = start->gVal + start->hVal;
    nodes[init] = start;
    open_queue.push(start);

    list<GroundedAction> actions;
    actions = computePath(goal);
    cout << "Num of states expanded: " << closed.size() << endl;

    return actions;
}



/** End **/




int main(int argc, char* argv[])
{
    // DO NOT CHANGE THIS FUNCTION
    char* filename = (char*)("Blocks.txt");
    if (argc > 1)
        filename = argv[1];

    cout << "Environment: " << filename << endl << endl;
    Env* env = create_env(filename);
    if (print_status)
    {
        cout << *env;
    }

    list<GroundedAction> actions = planner(env);

    cout << "\nPlan: " << endl;
    for (GroundedAction gac : actions)
    {
        cout << gac << endl;
    }

    return 0;
}