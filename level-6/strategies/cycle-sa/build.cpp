// build programs from FOUND cycles. usage: build input idx outprog cyclefile...
#define main base_main
#include "base.cpp"
#undef main
struct St{int o,a;};
char instr(St x, St y){ if(x.a!=y.a) return 'T'; if(y.o==x.o+1) return 'F'; if(y.o==x.o-1) return 'B'; return '-'; }
int main(int argc,char**argv){
    std::ifstream in(argv[1]); ccc::Reader r(in); Input input=parse(r);
    int idx=atoi(argv[2]); std::string outp=argv[3];
    const Geometry g(input.cases[idx].radius); const Target t(g,input.cases[idx]);
    Prog best; 
    for(int fi=4;fi<argc;++fi){
        std::ifstream f(argv[fi]); std::string line;
        while(std::getline(f,line)){
            if(line.rfind("FOUND",0)!=0) continue;
            std::istringstream ss(line); std::string w; int id,K; ss>>w>>w>>id>>w>>K;
            std::vector<std::array<St,6>> c(K);
            for(int j=0;j<K;++j){ std::getline(f,line); std::istringstream s2(line); for(int e=0;e<6;++e){ std::string tok; s2>>tok; sscanf(tok.c_str(),"%d,%d",&c[j][e].o,&c[j][e].a);} }
            if(id!=idx) continue;
            for(int ph=0;ph<K;++ph){
                Prog cyc; for(int j=1;j<=K;++j){ Line l; for(int e=0;e<6;++e) l[e]=instr(c[(ph+j-1)%K][e], c[(ph+j)%K][e]); cyc.push_back(l);} 
                // per emitter setup sequences
                std::array<int,6> d; int L=0;
                for(int e=0;e<6;++e){ d[e]=std::abs(c[ph][e].o)+c[ph][e].a; L=std::max(L,d[e]); }
                for(int extra=0;extra<=1;++extra){
                int LL=L+extra;
                for(int mask=0;mask<4096;++mask){
                    Prog p(LL,kNoop); bool skip=false;
                    for(int e=0;e<6;++e){ int v=(mask>>(2*e))&3; // bit0: align right, bit1: toggle first
                        St tg=c[ph][e]; std::vector<char> seq; char mv=tg.o>0?'F':'B';
                        if(tg.a && (v&2)) { seq.push_back('T'); for(int k=0;k<std::abs(tg.o);++k) seq.push_back(mv);} 
                        else { if(!tg.a && (v&2)) {skip=true;} for(int k=0;k<std::abs(tg.o);++k) seq.push_back(mv); if(tg.a) seq.push_back('T'); }
                        if(!tg.a && tg.o==0 && (v&1)) skip=true;
                        int st=(v&1)? LL-(int)seq.size() : 0;
                        for(size_t k=0;k<seq.size();++k) p[st+k][e]=seq[k];
                    }
                    if(skip) continue;
                    for(int rep=0;rep<t.count+3;++rep) for(auto&l:cyc) p.push_back(l);
                    int used=simulate(g,t,p);
                    if(used<0) continue;
                    p.resize(used);
                    if(best.empty()||p.size()<best.size()) best=p;
                }}
            }
        }
    }
    if(best.empty()){ fprintf(stderr,"idx %d: no valid program\n",idx); return 1; }
    std::ofstream o(outp); for(auto&l:best) for(int e=0;e<6;++e) o<<l[e]<<(e<5?' ':'\n');
    fprintf(stderr,"idx %d: best %zu\n",idx,best.size());
}
