(* ::Package:: *)

(*Basic linearization*)


BeginPackage["LinearizePackage`"];


Linearize::usage = "Linearize[f, x, x0] gives a first-order linearization of  f  with respect to  x  around  x0";


Begin["`Private`"];

Linearize[f_?VectorQ,x_?VectorQ,x0_?VectorQ]:=Module[{subs,f0,Jf0,dx,count},
	count=Length[x];
	subs=Table[x[[i]]->x0[[i]],{i,count}];
	f0=f/.subs;
	Jf0=D[f,{x}]/.subs;
	dx=Transpose[{x-x0}];
	Flatten[f0+Jf0.dx]
];
Linearize[f_/;!ListQ[f],args__]:=Module[{expr},
	expr=Linearize[{f},args];
	expr[[1]]
];

End[];


EndPackage[];
