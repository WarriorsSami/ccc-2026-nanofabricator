// SA search for steady-state cycles. usage: cyc input idx K subs(comma list of submission ticks) seconds seed outfile
#define main base_main
#include "base.cpp"
#undef main
#include <random>
#include <chrono>
using u64=uint64_t;
int R,K; std::vector<int> Q; u64 Pm=0,Hm=0; std::vector<std::vector<u64>> BM; // BM[e][o+R]
struct St{int o; int a;};
static inline u64 beamOf(int e,St s){ if(!s.a||std::abs(s.o)>R) return 0; return BM[e][s.o+R]; }
static inline bool adj(St x,St y){ if(x.a==y.a) return std::abs(x.o-y.o)<=1; return x.o==y.o; }
std::vector<char> isSub;
int evalCost(const std::vector<std::array<St,6>>& c, int* premature=nullptr){
    // c[j][e]
    static u64 E2[64],E3[64];
    for(int j=0;j<K;++j){u64 ones=0,twos=0,fours=0; for(int e=0;e<6;++e){u64 x=beamOf(e,c[j][e]); u64 c1=ones&x; ones^=x; u64 c2=twos&c1; twos^=c1; fours|=c2;} E2[j]=twos&~ones&~fours; E3[j]=fours|(twos&ones);}
    u64 st=0; int cost=0; int pre=0;
    for(int pass=0;pass<2;++pass) for(int j=0;j<K;++j){ st=(st&~(E2[j]|E3[j]))|E2[j];
        if(isSub[j]){ if(pass){cost+=__builtin_popcountll(Pm&~st)+__builtin_popcountll(Hm&st);} st&=~Pm; }
        else if(pass){ if((Pm&~st)==0 && (Hm&st)==0) pre++; }
    }
    if(premature)*premature=pre;
    return cost;
}
int main(int argc,char**argv){
    std::ifstream in(argv[1]); ccc::Reader r(in); Input input=parse(r);
    int idx=atoi(argv[2]); K=atoi(argv[3]);
    {std::string s=argv[4]; std::stringstream ss(s); std::string t; while(std::getline(ss,t,',')) Q.push_back(atoi(t.c_str()));}
    double secs=atof(argv[5]); unsigned seed=atoi(argv[6]); std::string outf=argv[7];
    const Geometry g(input.cases[idx].radius); const Target t(g,input.cases[idx]); R=g.R;
    for(int id:t.cells) Pm|=1ull<<id; for(int id=0;id<g.numCells();++id) if(t.isHalo[id]) Hm|=1ull<<id;
    BM.assign(6,std::vector<u64>(2*R+1,0)); for(int e=0;e<6;++e) for(int o=-R;o<=R;++o) for(int id:g.beamAt(e,o)) BM[e][o+R]|=1ull<<id;
    isSub.assign(K,0); for(int q:Q) isSub[q]=1;
    std::mt19937 rng(seed); auto t0=std::chrono::steady_clock::now();
    auto el=[&]{return std::chrono::duration<double>(std::chrono::steady_clock::now()-t0).count();};
    std::vector<St> all; for(int a=0;a<2;++a) for(int o=-R-1;o<=R+1;++o) all.push_back({o,a});
    int found=0; long long iters=0; int restarts=0; int bestEver=1e9;
    std::ofstream out(outf, std::ios::app);
    while(el()<secs){
        restarts++;
        std::vector<std::array<St,6>> c(K);
        for(int e=0;e<6;++e){ St s=all[rng()%all.size()]; for(int j=0;j<K;++j) c[j][e]=s; }
        int cur=evalCost(c); int best=cur;
        long long N=argc>8?atoll(argv[8]):200000; double T0=2.0,T1=0.05;
        for(long long it=0;it<N;++it){
            iters++;
            double T=T0*pow(T1/T0,(double)it/N);
            int e=rng()%6; int mv=rng()%10;
            std::vector<std::array<St,6>> old;
            St saved[64];
            for(int j=0;j<K;++j) saved[j]=c[j][e];
            bool ok=true;
            if(mv<7){ int j=rng()%K; St p=c[(j+K-1)%K][e], n=c[(j+1)%K][e]; St cand[32]; int nc=0;
                for(auto& s:all){ if((K==1||(adj(p,s)&&adj(s,n))) && !(s.o==c[j][e].o&&s.a==c[j][e].a)) cand[nc++]=s; }
                if(!nc){ok=false;} else c[j][e]=cand[rng()%nc];
            } else if(mv<9){ int d=(rng()&1)?1:-1; for(int j=0;j<K;++j){ c[j][e].o+=d; if(std::abs(c[j][e].o)>R+1) ok=false; } }
            else { // rotate in time
                int d=1+rng()%std::max(1,K-1); for(int j=0;j<K;++j) c[j][e]=saved[(j+d)%K]; }
            if(!ok){ for(int j=0;j<K;++j) c[j][e]=saved[j]; continue; }
            int nw=evalCost(c);
            if(nw<=cur || std::uniform_real_distribution<double>(0,1)(rng) < exp((cur-nw)/T)){ cur=nw; }
            else { for(int j=0;j<K;++j) c[j][e]=saved[j]; }
            if(cur<best) best=cur;
            if(cur==0){
                int pre; evalCost(c,&pre);
                // output cycle: states per tick
                out<<"FOUND idx "<<idx<<" K "<<K<<" subs "<<argv[4]<<" pre "<<pre<<"\n";
                for(int j=0;j<K;++j){ for(int e=0;e<6;++e) out<<c[j][e].o<<","<<c[j][e].a<<" "; out<<"\n"; }
                out.flush(); found++;
                break;
            }
        }
        bestEver=std::min(bestEver,best);
        if(found>=5) break;
    }
    fprintf(stderr,"idx %d K %d subs %s found %d bestEver %d restarts %d iters %lld time %.1f\n",idx,K,argv[4],found,bestEver,restarts,iters,el());
}
