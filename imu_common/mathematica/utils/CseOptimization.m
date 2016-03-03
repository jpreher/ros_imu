(* ::Package:: *)

Needs["ToMatlab`"];
Needs["Experimental`"];


BeginPackage["CseOptimization`"];

(*********CseOptimization: Optimize Expression via Common Subexpression Elimination (CSE) ***********)

CseWriteMatlab::usage = "CseWriteMatlab[name, expr] optimizes the expression 'expr' compound expressions by 
eliminating common subexpressions, then convert them into MATLAB systax and write to matlab file.

Options:
Directory -> output directory for file (default: '.')
Arguments -> arguments in MATLAB function, as symbols (default: {})
OptimizationLevel -> optimization level (0,1,2) for common subexpression elimination (default: 2)
PreStatement -> add additional statements before function body (like 'a = getrobotparameter(...)')";
CseOptimizeExpression::usage="CseOptimizeExpression[expr] eliminates common subexpressions in 'expr', 
and return a Block in Hold form.";

CseDecomposeBlock::usage="CseDecomposeBlock[Block] decomposes the 'Block' in hold into two parts: 
'vars' - a list of local variables, and 'code' - CompoundExpression of the 'Block' in Hold form";

CseReplaceVariable::usage="CseReplaceVariable[vars,code] replaces 'vars' in code to some 
readible symbols."

CseConvertToRule::usage="CseConvertToRule[code] replaces Set in expression 'code' to 
subsititution rules."

CseDeleteUnnecessoryExpr::usage="CseDeleteUnnecessoryExpr[code] deletes some sub expression 
in the 'code' that appears only once, and replace those subvariables with the 
original expressions in the following expression (it should have only one expression
that use the subvaribale.)";

CseSequenceExpression::usage="CseSequenceExpression[code] decomposes the compound expression 'code into two parts and
return 'seq' - sequenced expression except the last one.";
CseFinalExpression::usage="CseFinalExpression[code] decomposes the compound expression 'code into two parts and
return 'final' - the last expression.";

CseOptimizeToRule::usage="CseOptimizeToRule[expr] optimizes the expression 'expr' compound expressions by 
eliminating common subexpressions, then return a subsititution rule in Hold form.";

CseWriteCpp::usage="CseWriteCpp[name,expr,options] optimizes expressions using CSE and write the resulting statements
into cpp (mex) code."

ExportWithGradient::usage="ExportWithGradient[name_,expr_,vars_] exports the expr and its Jacobian w.r.t. vars in two
seperate files f_name and J_name."


ToMatrixForm::usage="ToMatrixForm[expr] converts expr to Matrix form (two dimensional tensor)."
ToVectorForm::usage="ToVectorForm[expr] converts expr to Vector form (one dimensional tensor)."


Begin["`Private`"]


ExportWithGradient[name_,expr_,vars_]:=
Block[
	{gradExpr,fvars,statesubs},
	fvars=Flatten[vars];
	gradExpr=ToVectorForm[Grad[Flatten[expr],fvars]];
	statesubs=Dispatch@Join[((fvars[[#+1]]-> HoldForm@Global`var[[#]]&)/@(Range[Length[fvars]]-1))];
	SetOptions[CseWriteCpp,
		ArgumentLists->{Global`var},
		ArgumentDimensions-> {{Length[fvars],1}},
		SubstitutionRules-> statesubs
	];
	CseWriteCpp["f_"<>name,{expr}];
	CseWriteCpp["J_"<>name,{gradExpr}];
];
ExportWithGradient[name_,expr_,vars_,consts_]:=
Block[
	{gradExpr,fvars,statesubs,fconsts},
	fvars=Flatten[vars];
	fconsts=Flatten[consts];
	gradExpr=ToVectorForm[Grad[Flatten[expr],fvars]];
	statesubs=Dispatch@Join[Flatten[{((fvars[[#+1]]-> HoldForm@Global`var[[#]]&)/@(Range[Length[fvars]]-1)),((fconsts[[#+1]]-> HoldForm@Global`auxdata[[#]]&)/@(Range[Length[fconsts]]-1))}]];
	SetOptions[CseWriteCpp,
		ArgumentLists->{Global`var,Global`auxdata},
		ArgumentDimensions-> {{Length[fvars],1},{Length[fconsts],1}},
		SubstitutionRules-> statesubs
	];
	CseWriteCpp["f_"<>name,{expr}];
	CseWriteCpp["J_"<>name,{gradExpr}];
];



ToVectorForm[expr_?MatrixQ]:=Flatten@(expr\[Transpose]);
ToVectorForm[expr_?VectorQ]:=expr;
ToVectorForm[expr_/;!ListQ[expr]]:=Flatten@expr;

ToMatrixForm[expr_?MatrixQ]:=expr;
ToMatrixForm[expr_?VectorQ]:=Transpose[{expr}];
ToMatrixForm[expr_]:={expr};


CseOptimizeExpression[expr_,OptionsPattern[]]:=
	Block[{optExpr},
	optExpr = Experimental`OptimizeExpression[expr,OptimizationLevel->OptionValue[OptimizationLevel]];
	optExpr
	];
Options[CseOptimizeExpression]={OptimizationLevel-> 1};

CseDecomposeBlock[block_]:=
	ReleaseHold[(Hold@@block)/.Verbatim[Block][vars_,seq_]:>{vars,Hold[seq]}];

CseReplaceVariable[vars_,code_]:=
    Block[{nvars},
	nvars=Dispatch[MapIndexed[#1-> ToExpression["t"<>ToString@@#2]&,vars]];
	{vars,code}/.nvars];


CseWriteCpp[name_String,expr_,OptionsPattern[]]:=
	Block[{argins,arginDims,csubs,cFile,hFile,argoutDims,funcLists,tpl,hdr,seqs,finals,lvars,optStatement,assoc},
		(*Obtain basic information*)
		argins = OptionValue[ArgumentLists];
		arginDims = OptionValue[ArgumentDimensions];
		csubs=OptionValue[SubstitutionRules];
		cFile = FileNameJoin[{OptionValue[Directory],name <> ".cc"}];
		hFile = FileNameJoin[{OptionValue[Directory],name <> ".hh"}];
		argoutDims=Join[Dimensions/@(ToMatrixForm/@expr)];
		funcLists=StringInsert[ToString/@Range[Length[argoutDims]],"output",1];
		optStatement=Block[{vexpr,oexpr,seq,vars,code,subcode,final,statement,result},
			Table[
			vexpr=ToVectorForm[expr[[i]]];
			oexpr=CseDecomposeBlock[CseOptimizeExpression[vexpr]];
			If[ListQ[First[oexpr]],
				{vars,code}=CseReplaceVariable[First[oexpr],Last[oexpr]];
				subcode=code/.csubs;
				seq=Map[Hold,N[subcode,15],{2}]/.Hold[CompoundExpression[seq__,f_]]:>{seq};
				final=Map[Hold,N[subcode,15],{2}]/.Hold[CompoundExpression[seq__,f_]]:>{f};
				statement=StringReplace[ToString[CForm[#]],"Hold("~~ShortestMatch[a___]~~")":>a]&/@seq;
				result=ToString[CForm[ReleaseHold@final]];
				,
				vars={"_NotUsed"};
				statement={"NULL"};
				result=ToString[CForm[oexpr/.csubs]];];
				{vars,statement,result}
				,{i,Length[funcLists]}
			]
		];
		lvars=optStatement[[All,1]];
		seqs=optStatement[[All,2]];
		finals=optStatement[[All,3]];
		tpl=FileTemplate[FileNameJoin[{".",OptionValue[TemplateFile]}]];
		hdr=FileTemplate[FileNameJoin[{".",OptionValue[TemplateHeader]}]];
		assoc=<|"name"->name,"argins"-> argins,"argouts"-> funcLists,"arginDims"-> arginDims,"argoutDims"-> argoutDims,"final"-> finals,"lvars"-> lvars,"statements"-> seqs,"namespace"->OptionValue[Namespace],"behavior"->OptionValue[Behavior]|>;
		FileTemplateApply[tpl,assoc,cFile];
		FileTemplateApply[hdr,assoc,hFile];
	];
Options[CseWriteCpp]={Directory->".",ArgumentLists->{},ArgumentDimensions->{},SubstitutionRules->{},TemplateFile->FileNameJoin[{"template","template.cc"}],TemplateHeader->FileNameJoin[{"template","template.hh"}],Namespace->"durus_3d_expr",Behavior->"basic"};


CseConvertToRule[code_]:= code/.Set-> Rule;

CseDeleteUnnecessoryExpr[code_]:=
	Block[{unvars},
	unvars=Cases[Cases[code,_Symbol,Infinity]//Tally,{_,2}][[All,1]];
	Verbatim[Rule][Alternatives@@unvars,_]//DeleteCases[code,#,Infinity]//.Cases[code,#,Infinity]&
	];


CseSequenceExpression[code_]:= code/.Hold[CompoundExpression[s___,f_]]:>s;
CseFinalExpression[code_]:= code/.Hold[CompoundExpression[s___,f_]]:>f;


CseOptimizeToRule[expr_,OptionsPattern[]]:=
	Block[{vars,code,subCode,seq,final},
	{vars,code} = CseDecomposeBlock[CseOptimizeExpression[expr,OptimizationLevel->OptionValue[OptimizationLevel]]];
	subCode = CseDeleteUnnecessoryExpr[CseConvertToRule[CseReplaceVariable[vars,code]]];
	seq = CseSequenceExpression[subCode];
	final = CseFinalExpression[subCode];
	Hold[final/.seq]
];
(*TODO:fix the output*)
Options[CseOptimizeToRule]={OptimizationLevel->2};	


CseWriteMatlab[name_String,expr_,OptionsPattern[]]:=
	Block[{vars,code,subCode,seq,final,body, file, args, argsString, outputName, extra,comment},
	{vars,code} = CseDecomposeBlock[CseOptimizeExpression[expr,OptimizationLevel->OptionValue[OptimizationLevel]]];
	subCode = CseDeleteUnnecessoryExpr[CseConvertToRule[CseReplaceVariable[vars,code]]];
	seq = CseSequenceExpression[subCode];
	final = CseFinalExpression[subCode];

	outputName = "x_" <> name;
	file = FileNameJoin[{OptionValue[Directory], name <> ".m"}];
	extra = (If[# != "", # <> "\n", ""] &) @ OptionValue[PreStatement];
	args = OptionValue[Arguments];
	argsString = StringJoin[Riffle[ToString /@ args, ", "]];
	comment = "%"<>name<>"\n%    "<>outputName<>" = "<>name<>"("<>argsString <> ")\n\n"<>
		"% This function was generated by Mathematica Common \n% Subexpression Eliminating Package (CseOptimization)\n% " <> DateString[];
	body = "function [" <> outputName <> "] = " <> name <> "(" <> argsString <> ")\n" <> comment <>"\n\n"<> extra <> ToMatlab`RulesToMatlab[List[seq]]<>
		outputName<>"="<>ToString[ToMatlab`ToMatlab[final]]<>"\nend %function";
	Block[{f = OpenWrite[file]},
		WriteString[f, body];
		Close[f];
	];
];

Options[CseWriteMatlab]={Directory->".",Arguments->{},OptimizationLevel->2,PreStatement->""};	

End[]

EndPackage[]



