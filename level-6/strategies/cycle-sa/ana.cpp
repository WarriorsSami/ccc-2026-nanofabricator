#define main base_main
#include "base.cpp"
#undef main
std::vector<Prog> readPrograms(const std::string& path) {
    std::ifstream f(path);
    std::vector<Prog> progs(1);
    std::string line;
    while (std::getline(f, line)) {
        if (line.find_first_not_of(" \t\r") == std::string::npos) {
            if (!progs.back().empty()) progs.emplace_back();
            continue;
        }
        std::istringstream ss(line);
        Line l = kNoop;
        std::string tok;
        for (int e = 0; e < 6 && ss >> tok; ++e) l[e] = tok[0];
        progs.back().push_back(l);
    }
    if (progs.back().empty()) progs.pop_back();
    return progs;
}
int main(int argc,char**argv){
    std::ifstream in(argv[1]); ccc::Reader r(in); Input input=parse(r);
    auto progs=readPrograms(argv[2]);
    for(size_t k=0;k<input.cases.size();++k){
        const Geometry g(input.cases[k].radius); const Target t(g,input.cases[k]);
        // simulate printing submission ticks
        const Prog& prog=progs[k];
        std::array<int,6> off{}; std::array<bool,6> act{}; std::vector<char> m(g.numCells(),0); std::vector<int> hits(g.numCells(),0);
        int acc=0; std::vector<int> ticks;
        for(int tick=0;tick<(int)prog.size();++tick){
            for(int e=0;e<6;++e){switch(prog[tick][e]){case 'F':++off[e];break;case 'B':--off[e];break;case 'T':act[e]=!act[e];break;}
                if(off[e]>3*g.R)off[e]-=g.ringSize; if(off[e]<=-3*g.R)off[e]+=g.ringSize;}
            std::vector<int> touched;
            for(int e=0;e<6;++e){if(!act[e]||std::abs(off[e])>g.R)continue; for(int id:g.beamAt(e,off[e])) if(hits[id]++==0)touched.push_back(id);}
            for(int id:touched){if(hits[id]==2)m[id]=1;else if(hits[id]>=3)m[id]=0;hits[id]=0;}
            bool pr=true; for(int id:t.cells)pr=pr&&m[id];
            if(pr)for(int id=0;id<g.numCells()&&pr;++id)pr=!(t.isHalo[id]&&m[id]);
            if(pr){for(int id:t.cells)m[id]=0; ticks.push_back(tick+1); if(++acc>=t.count)break;}
        }
        printf("%zu R=%d P=%zu H=%d cnt=%d len=%zu subs:",k,g.R,t.cells.size(),(int)std::count(t.isHalo.begin(),t.isHalo.end(),1),t.count,prog.size());
        int prev=0; for(int x:ticks){printf(" %d",x-prev);prev=x;} printf("\n");
    }
}
