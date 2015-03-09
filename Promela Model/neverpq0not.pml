never  {    /* [] (p -> !(q)) */
accept_init:
T0_init:
	do
	:: (((! ((p))) || (! ((q))))) -> goto T0_init
	od;
}

