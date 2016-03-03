(* ::Package:: *)

BeginPackage["AmberCpp`", {"AmberShared`"}];


(* ::Section:: *)
(*C++ *)


WriteCpp::usage=
 "Generates a C++ source code file with the passed expression.";


WriteInvCpp::usage=
 "Generates a C++ source code file with the passed expression which is inverted.";


MakeDynamicsCppFromTemplate::usage="Generates a C++ source code file for computing the dynamics."


Begin["`Private`"]


nbdir=Global`nbdir;
builddir=Global`builddir;


ne=Global`ne;
np=Global`np;
nx=Global`nx;
Xe=Global`Xe;


Cstatesubs = Join[
	((Xe[[#+1,1]]->HoldForm@Global`x[[#]]&)/@(Range[Length[Xe]]-1)),
	((Global`p[#+1]->HoldForm@Global`p[[#1]]&)/@(Range[np]-1))
];


getCFormNoPowers[expr_] := Module[{times}, Apply[Function[code, Hold[CForm[code]], HoldAll],Hold[#]&[expr/.x_Symbol^y_Integer/;3>y>1:>times@@Table[x,{y}]]/.times->Times]];


ToCForm[expr_?MatrixQ]:=Flatten@(expr\[Transpose])/.Cstatesubs;
ToCForm[expr_?VectorQ[expr]]:=expr/.Cstatesubs;
ToCForm[expr_/;!ListQ[expr]]:=Flatten@expr/.Cstatesubs;


MakeCppFromTemplate[var_,file_?StringQ,template_?StringQ]:=Module[{foo,baz},
	baz=If[ArrayQ[var],var,{var}];
	baz=If[VectorQ[baz],{baz},baz];
	{nRows,nCols}=Dimensions[baz];
	
	matrixCform=ToCForm[baz];
	foo=Splice[
		FileNameJoin[{nbdir,"templates",template}],
		FileNameJoin[{builddir,file<>".cc"}]
	];
	Clear[nRows,nCols,matrixCform];
	foo
]


MakeDynamicsCppFromTemplate[M_, C_, G_]:=Module[{},
	{nRows,nCols}=Dimensions[M];
	
	MCform=ToCForm[M];
	CCform=ToCForm[C];
	GCform=ToCForm[G];

	foo=Splice[
		FileNameJoin[{nbdir,"templates","dynamics.mcc"}],
		FileNameJoin[{builddir,"f_vec.cc"}]
	];
	Clear[nRows,nCols,MCform];
	Clear[nRows,nCols,CCform];
	Clear[nRows,nCols,GCform];
]


WriteCpp[var_?MatrixQ,file_?StringQ] := MakeCppFromTemplate[var,file,"mex_matrix.mcc"]
WriteCpp[var_?VectorQ,file_?StringQ] := MakeCppFromTemplate[var,file,"mex_matrix.mcc"]
WriteCpp[var_/;!ArrayQ[var],file_?StringQ] := MakeCppFromTemplate[{{var}},file,"mex_matrix.mcc"]


WriteInvCpp[var_?MatrixQ,file_?StringQ] := MakeCppFromTemplate[var,file,"mex_inv_matrix.mcc"]
WriteInvCpp[var_?VectorQ,file_?StringQ] := MakeCppFromTemplate[var,file,"mex_inv_matrix.mcc"]
WriteInvCpp[var_/;!ArrayQ[var],file_?StringQ] := MakeCppFromTemplate[{{var}},file,"mex_inv_matrix.mcc"]


End[]


EndPackage[]
