
#define NB       -6
#define NM	 -4
#define NS	 -2
#define ZO	 0
#define PS	 2
#define PM	 4
#define PB	 6
 
 //πÊ‘Úø‚
static const float ruleKp[7][7]={
	PB,	PB,	PM,	PM,	PS,	PS,	ZO,
	PB,	PB,	PM,	PM,	PS,	ZO,	ZO,
	PM,	PM,	PM,	PS,	ZO,	NS,	NM,
	PM,	PS,	PS,	ZO,	NS,	NM,	NM,
	PS,	PS,	ZO,	NS,	NS,	NM,	NM,
	ZO,	ZO,	NS,	NM,	NM,	NM,	NB,
	ZO,	NS,	NS,	NM,	NM,	NB,	NB
};
 
static const float ruleKi[7][7]={
	NB,	NB,	NB,	NM,	NM,	ZO,	ZO,
	NB,	NB,	NM,	NM,	NS,	ZO,	ZO,
	NM,	NM,	NS,	NS,	ZO,	PS,	PS,
	NM,	NS,	NS,	ZO,	PS,	PS,	PM,
	NS,	NS,	ZO,	PS,	PS,	PM,	PM,
	ZO,	ZO,	PS,	PM,	PM,	PB,	PB,
	ZO,	ZO,	PS,	PM,	PB,	PB,	PB
};
 
static const float ruleKu[7][7]={
	NB,	NB,	NM,	NM,	NS,	NS,	PS,
	NB,	NM,	NM,	NS,	NS,	ZO,	PS,
	NB,	NM,	NS,	NS,	ZO,	PS,	PM,
	NM,	NS,	NS,	ZO,	PS,	PS,	PM,
	NM,	NS,	ZO,	PS,	PS,	PM,	PB,
	NS,	ZO,	PS,	PS,	PM,	PM,	PB,
	NS,	PS,	PS,	PM,	PM,	PB,	PB
};

static const float speedruleKp[7][7]={
	PB,	PB,	PM,	PS,	NS,	NM,	ZO,
	PB,	PM,	PS,	PS,	PS,	ZO,	PS,
	PB,	PM,	PS,	PS,	PS,	PS,	PS,
	PM,	PM,	PS,	PS,	PS,	PM,	PM,
	PS,	PS,	PS,	PS,	PS,	PM,	PB,
	PS,	ZO,	PS,	PS,	PS,	PM,	PB,
	ZO,	NM,	NS,	PS,	PM,	PB,	PB
};

static const float speedruleKi[7][7]={
	NB,	NM,	ZO,	PB,	PB,	ZO,	NM,
	NB,	NM,	ZO,	PB,	PM,	ZO,	NM,
	NB,	NS,	PS,	PB,	PM,	ZO,	NM,
	NB,	NS,	PS,	PB,	PS,	NS,	NB,
	NM,	ZO,	PM,	PB,	PS,	NS,	NB,
	NM,	ZO,	PM,	PB,	ZO,	NM,	NB,
	NM,	ZO,	PB,	PB,	ZO,	NM,	NB
};