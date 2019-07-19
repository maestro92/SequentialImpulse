#ifndef MODEL_ENUM
#define MODEL_ENUM


using namespace std;
namespace ModelEnum
{
	enum E
	{
		none = 0,
		custom, 
		player,
		healthbar,
		xyzAxis,
        tree,
        cube,
        circle,
        arrow,
    //    centeredQuad,
    //    centeredQuadOutline,
        unitCenteredQuad,
        unitCenteredQuadOutline,

        woodenBox,
        legoMan,
        ground,
        animatedLegoMan,

        NUM_MODELS
	};
}

#endif