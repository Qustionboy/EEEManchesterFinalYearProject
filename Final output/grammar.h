#ifndef GRAMMAR_H
#define GRAMMAR_H

#include <string>
#include <vector>
#include <ctime>
#include <cstdlib>
#include "random_generator.h"

class Grammar {
public:
    std::string Gname;  // Grammar name
    int level = 6;      // iteration times, layer number
    std::string Startgrammar = "C";
    std::vector<std::string> grammarlist;
    std::string rule;   // iterated rule
    RandomGenerator Rand;


    Grammar() {}   // Constructor
    void clear();  // Clear all the grammar
    void initialGrammarList();
    void iteration();  // Iterate
    void setGrammarName(const std::string& ref);
    void setLevel(int num);
    std::string getGrammarName() const;
    int getLevel() const;
    std::string getRule() const;
};

#endif // GRAMMAR_H
