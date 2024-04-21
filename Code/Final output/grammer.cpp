#include "grammar.h" 
#include "grammar.h" 


void Grammar::clear() {
    rule.clear();
}

void Grammar::initialGrammarList() { 
    grammarlist.push_back("FB[*AUBOAUBQAWBT]AUB[/UOUPWT]AU[^%UOUPUX]AU[&$UQUPUX]AC");
    grammarlist.push_back("FB[/%AUBOAUBQAWBT]AUB[*$UOUPWT]AU[&%UUPUX]AU[^$UQUPUX]AC");
    grammarlist.push_back("FB[*%%%%%AUBOAUBQAWBT]AUB[/%%%%UOUPWT]AU[^%%%%%%UOUPUX]AU[&%%%%UQUPUX]AC");
    grammarlist.push_back("FB[/$$$$$UOUT]AUB[*%%%%%UQUPUX]AU[^%%%%%UPUT]AU[&%%%%%%%%%%%UPUX]AC");
    grammarlist.push_back("FB[/%%FT]AUB[*%%FT]AU[^UP]AU[&UO]AC");
    grammarlist.push_back("FB[/%%%%UO]AUB[*%%%%UQ]AU[^%%UQ]AU[&%%UO]AC");
}

void Grammar::iteration() {
    std::string temprule = Startgrammar;
    initialGrammarList();
    int num_grammar = 6;
    int berryCount = 0;

    for (int i = 1; i <= level; i++)
    {
        if (i == level) {
            if (rand() % 2 == 0) { 
                temprule += "FF[^%%%UUX][&%%%UUX]F[X]";
            }
            else {
                temprule += "FF[^%%%UUX][&%%%UUX]F";
            }
        }
        int curlen = temprule.length();
        int j = 0;
        int last_idx = 0;
        int initial_idx = 0;
        while (j < curlen)
        {
            if (temprule[j] == 'C')
            {
                if (i == 1)
                {
                    int idx = Rand.getRandomInt(0, 32767) % 2;
                    initial_idx = idx + 1;
                    last_idx = idx;
                    rule += grammarlist[idx];
                    j++;
                }
                else
                {
                    int idx = initial_idx + Rand.getRandomInt(0, 32767) % (num_grammar - initial_idx);
                    if (last_idx == idx) idx = initial_idx + Rand.getRandomInt(0, 32767) % (num_grammar - initial_idx); 
                    last_idx = idx;
                    rule += grammarlist[idx];
                    j++;

                }
            }


            else if (temprule[j] == 'O')
            {
                rule += "[%%%*&UX]U[&/UX]"; // sub for berry to grow 生成两个枝加葡萄STYLE1
                j++;
            }
            else if (temprule[j] == 'P')
            {
                rule += "[*&UX]U[^/$UX]"; // sub for berry to grow 生成两个枝加葡萄STYLE2
                j++;
            }
            else if (temprule[j] == 'Q')
            {
                rule += "[%*&UX]U[^/UX]"; // sub for berry to grow 生成两个枝加葡萄STYLE3
                j++;
            }
            else if (temprule[j] == 'T')
            {
                rule += "[*$$UX]U[^$$$$UX][&&WX]"; // sub for berry to grow 生成三个枝加葡萄
                j++;
            }



            else
            {
                rule += temprule[j];
                j++;
            }
        }
        temprule = rule;
        rule.clear();
    }

    int curlen = temprule.length();//获得当前rule的长度
    int j = 0;
    while (j < curlen)
    {
        if (temprule[j] == 'O')
        {
            rule += "[%%%*&UX]U[&/UX]"; // STYLE1
            j++;
        }
        else if (temprule[j] == 'P')
        {
            rule += "[*&UX]U[^/$UX]"; // STYLE2
            j++;
        }
        else if (temprule[j] == 'Q')
        {
            rule += "[%*&UX]U[^/UX]"; // STYLE3
            j++;
        }
        else if (temprule[j] == 'T')
        {
            rule += "[*$$UX]U[^$$$$UX][&&WX]"; // sub for berry to grow 生成三个枝加葡萄
            j++;
        }
        else
        {
            rule += temprule[j];
            j++;
        }
    }
    temprule = rule;
    rule = temprule;//final output
}

void Grammar::setGrammarName(const std::string& ref) {
    Gname = ref;
}

void Grammar::setLevel(int num) {
    level = num;
}

std::string Grammar::getGrammarName() const {
    return Gname;
}

int Grammar::getLevel() const {
    return level;
}

std::string Grammar::getRule() const {
    return rule;
}