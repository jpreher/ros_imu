(* ::Package:: *)

BeginPackage["AmberShared`"];


(* ::Section:: *)
(*Exports*)


ParallelSimplify::usage="Simplifies a matrix or vector in parallel."


ParallelFullSimplify::usage="Fully simplifies a matrix or vector in parallel."


J::usage="Computes the Jacobian of a quantity with respect to the given coordinates."


Lie::usage="Computes the Lie derivative of the passed quantity with respect to the passed vector field in the passed coordinates."


CrossProd::usage="Extracts the cross product vector form which is dual to the passed skew-symmetric matrix."


Vec::usage="Vec[x]  Turn arbitrary list into vector";


ToExpressionEx::usage="ToExpressionEx[expr]  loosely converts any string types in an 0- to n-dimensional list to an expression.";


RationalizeEx::usage="RationalizeEx[expr]  loosely rationalize any expression to an arbitrary precision";


RationalizeAny::usage="RationalizeAny[value]  convert `value` to an expression and use RationalizeEx";


BlockDiagonalMatrix::usage="Create block diagonal matrx.";


EnsureDirectoryExists::usage="Ensure directory exists, if not create one.";


CRoundEx::usage="For eliminating those pesky small numbers in rotation matrices";


(* ::Section:: *)
(*Matrix Functions*)


Begin["`Private`"]


J[h_,x_]:=D[Flatten[h],{Flatten[x]}]
Lie[h_,x_,f_]:=J[h,x].f
CrossProd[\[CapitalOmega]_?MatrixQ]:={\[CapitalOmega][[3,2]],\[CapitalOmega][[1,3]],\[CapitalOmega][[2,1]]};
Vec[x_]:=Transpose[{Flatten[x]}];


ToExpressionEx[value_]:=Module[{result},result=If[StringQ[value],ToExpression[value],If[ListQ[value],Map[If[StringQ[#],ToExpression[#],#]&,value,-1],value]];
Return[result];];
RationalizeEx[expr_]:=Rationalize[expr,0];
RationalizeEx[expr_List]:=Map[RationalizeEx,expr,-1];
RationalizeAny[expr_]:=RationalizeEx[ToExpressionEx[expr]];


(* ::Section:: *)
(*Simplification*)


ParallelSimplify[A_?MatrixQ]:=ParallelTable[Simplify[A[[i,j]]],{i,Dimensions[A][[1]]},{j,Dimensions[A][[2]]}]
ParallelSimplify[A_?VectorQ]:=ParallelTable[Simplify[A[[i]]],{i,Length[A]}]
ParallelSimplify[A_]:=Simplify[A]

ParallelFullSimplify[A_?MatrixQ]:=ParallelTable[FullSimplify[A[[i,j]]],{i,Dimensions[A][[1]]},{j,Dimensions[A][[2]]}]
ParallelFullSimplify[A_?VectorQ]:=ParallelTable[FullSimplify[A[[i]]],{i,Length[A]}]
ParallelFullSimplify[A_]:=FullSimplify[A]


(* For eliminating those pesky small numbers in rotation matrices *)
CRound[expr_, n_:-5] := If[NumberQ[expr], Round[expr, 10^n], expr];
CRoundEx[expr_, n_:-5] := Map[CRound[#, n]&, expr, {-1}];


(*From:http://mathworld.wolfram.com/BlockDiagonalMatrix.html*)
BlockDiagonalMatrix[b:{__?MatrixQ}]:=
	Module[{r,c,n=Length[b],i,j},
		{r,c}=Transpose[Dimensions/@b];
		ArrayFlatten[
			Table[
				If[i==j,b[[i]],ConstantArray[0,{r[[i]],c[[j]]}]]
				,
				{i,n},{j,n}
			]
		]
	];


EnsureDirectoryExists[dir_?StringQ]:=
	Module[{pieces,cur},
		pieces=FileNameSplit[dir];
		Table[
			cur=FileNameJoin[pieces[[1;;i]]];
			If[!DirectoryQ[cur],
				CreateDirectory[cur];
			];
			,
			{i,Length[pieces]}
		];
	];


End[]
EndPackage[]
