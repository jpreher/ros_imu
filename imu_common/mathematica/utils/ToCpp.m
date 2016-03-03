(* ::Package:: *)

BeginPackage["ToCpp`"];

(* Needs CommonSubexpressionElimination, but because of path difficulties, will just have end user
include it themselves *)

(*
TODO:
Convert Power(x,2) to "x * x"

Make sure that only known symbols are present in an expression?
Precision - especially for integers!
X For large integers, it's converting them to floating-point values... Bad!
- Actually, this is a little more complicated in C.
*)

LoadTemplate::usage =
"LoadTemplate[tplBase, includeDir = None]  load template base .h and .cpp to generate resulting files.
includeDir - ONLY FOR TEMPLATE. If None, will be same directory as source file. Otherwise, will concat this dir and basename of source file";

ToCppFunc::usage =
"ToCppFunc[funcName, argSet, expr, preStatement : \"\"]  generates C++ code with Eigen library for given expression.
argSet - {{argName, argType}, ...} in C++
Note that all fractions are 'numericized' and array indices (i.e. q[2]) are converted from 1..n to 0..n-1 (thus q[i] -> q[i-1])
preStatement - Statement inserted before. Good for Common Subexpression Elimination. You handle your own newlines and indentation.
Returns {function declaration, function definition}";

ToCppGenerateFiles::usage =
"ToCppGenerateFiles[fileBase, namespace, defSet]  genenerate header and source file for C++ Eigen usage.
defSet = {arguments for ToCppFunc[...] - see help for this}
fileBase - basename for .h and .cpp files to be generated";

ToCppExpr::usage = "
ToCppExpr[name, expr, indent = '\t']  converts expr to C++ Eigen form assigned to variable name.
THIS DOES NOT ADJUST INDICES.
Type (Matrix vs. Vector vs. Scalar) is automatically inferred.";

CRoundIndexSubs::usage = 
"CRoundIndexSubs[symbols]  make substitution rules to round and subtract 1 from all 'function' indices of a given symbol. i.e. x[5] -> x[4], a[10, 7] -> a[9, 6]";


Begin["`Private`"];

ExtraUtil`ContextAlias[{"StringImplode", "ExportText", "PseudoSplice"}];

CRoundIndices[indices_List] := Table[If[NumberQ[index], Round[index] - 1, index], {index, indices}];
CRoundIndexSubs[symbols_List] := Map[#[indices__]:>#@@CRoundIndices[{indices}]&@#&, symbols];

tplBase = None;
includeDir = None;

LoadTemplate[tplBaseSub_, includeDirSub_: None] := 
  Module[{}, tplBase = tplBaseSub;
   includeDir = includeDirSub;
   tplHeader = Import[tplBase <> ".hpp", "Text"];
   tplSource = Import[tplBase <> ".cpp", "Text"];
   ];


ToCppExpr[name_String, expr_, indent_: "\t", symbols_:{}] := 
  Module[{temp, strOut, type, dims, strSize, matForm, strExpr, 
    newLine, strExtra, indexSubs},
   indexSubs = CRoundIndexSubs[symbols];
   temp = N[expr] /. indexSubs;
   (*Vectors, for our use, will be row matrices*)
   (*If[VectorQ[
   temp],temp=Transpose[{temp}]];*)
   
   If[ListQ[temp] && Length[temp] == 0, temp = {{}}];
   If[VectorQ[temp], temp = Transpose[{temp}]];
   strOut = "";
   (*Need to figure out how to matrix coordinates for given matrix \
types, use 0..n-1 instead of 1..n*)
   newLine = "\n" <> indent;
    If[MatrixQ[temp],
        (*Matrix*)
        type = "Eigen::MatrixXd";
        dims = Dimensions[temp];
        (* Handle zero-size matrices *)
        If[dims[[2]] > 0,
            strSize = StringImplode[dims, ", "];
            matForm = Map[CForm, temp, {Length[dims]}];
            (* Should it be flattened, or keep the shape? For huge expressoins, definitely flattened *)
            strExpr = StringImplode[Flatten[matForm], "," <> newLine <> "\t ", "(``)"];    
            strExtra = StringImplode[{newLine, name, " << ", strExpr, ";"}];
        ,
            strSize = "0, 0";
            strExtra = "";
        ];
        strOut = StringImplode[{type, " ", name, "(", strSize, ");", strExtra}];
    ,
        (*Scalar*)
        type = "double";
        strExpr = CForm[temp];
        (*strOut=SprintF["`1` `2` = (`3`);\n",type,name,strExpr];*)
        
        strOut = StringImplode[{type, " ", name, " = (", strExpr, ");"}];
    ];
   Return[{strOut, type}];
   ];

(* For additional assignments, to allow Common Subexpression Elimination expression sets*)
ToCppExpr[name_String, exprSet:{rules_,expr_}?ExtraUtil`ExpressionSetQ, indent_:"\t", symbols_:{}] :=
Module[{strOut, strSubExprs, strExpr, type},
	strSubExprs = Table[ToCppExpr[ToString[rule[[1]]], rule[[2]], indent, symbols][[1]], {rule, rules}];
	strOut = "";
	If[Length[strSubExprs] > 0,
		strOut = StringImplode[strSubExprs, "\n" <> indent] <> "\n" <> indent;
	];
	{strExpr, type} = ToCppExpr[name, expr, indent, symbols];
	strOut = strOut <> strExpr;
	Return[{strOut, type}];
];


ToCppFunc[funcName_, argSet_, expr_, preStatement_: ""] := 
 Module[{strVar, strExpr, ctype, strDecl, strDef, strArgs, 
   indexSubs},
  (*
  Function type is inferred;
  argSet = {{symbol, ctype},...}
  *)
  strVar = "value";
  {strExpr, ctype} = ToCppExpr[strVar, expr, "\t", argSet[[;;, 1]]];
  strArgs = 
   Table[StringImplode[{"const ", arg[[2]], " &", arg[[1]]}], {arg, 
      argSet}] // StringImplode[#, ", "] &;
  strDecl = 
   StringImplode[{ctype, " " , funcName, "(", strArgs, ")"}];
  strDef = 
   StringImplode[{strDecl, "\n{\n\t", preStatement, strExpr, 
     "\n\treturn ", strVar, ";\n}"}];
  Return[{strDecl, strDef}];
  ];


mathematicaIncludeDir = FileNameJoin[{$InstallationDirectory, "SystemFiles", "IncludeFiles", "C"}];

ToCppGenerateFiles[fileBase_, namespace_, defSet_] := 
  Module[{pair, pairs, strHeaders, strBodies, strHeaderBody, 
    strBodyBody, subs, fileHeader, fileSource, fileBaseTrue, 
    fileHeaderForTemplate}, 
   If[tplBase == None, 
    Throw["Please call 'LoadTemplate' before using this function."]];
   Print["C++ Conversion:"];
   pairs = Table[
     Print["\t\t" <> def[[1]]];
     pair = ToCppFunc @@ def;
     pair, {def, defSet}];
   {strHeaders, strBodies} = Transpose[pairs];
   strHeaderBody = StringImplode[strHeaders, ";\n\n"] <> ";";
   strBodyBody = StringImplode[strBodies, "\n\n"];
   fileHeader = fileBase <> ".hpp";
   fileSource = fileBase <> ".cpp";
   fileBaseTrue = FileNameSplit[fileBase][[-1]];
   fileHeaderForTemplate = 
    If[includeDir == None, fileBaseTrue, 
      FileNameJoin[{includeDir, fileBaseTrue}]] <> ".hpp";
   subs = {{"EXPR_HEADER", fileHeaderForTemplate}, {"EXPR_INCLUDE_GUARD", 
      "_H_" <> 
       ToUpperCase[namespace <> "_" <>
        StringReplace[fileBaseTrue, "." -> "_"]]}, {"EXPR_NAMESPACE", 
      namespace}, {"EXPR_DECLARATIONS", strHeaderBody}, {"EXPR_DEFINITIONS", 
      strBodyBody}};
   txtHeader = PseudoSplice[tplHeader, subs];
   txtSource = PseudoSplice[tplSource, subs];
   Print["\t", fileHeader, "\n\t", fileSource];
   Export[fileHeader, txtHeader, "Text"];
   Export[fileSource, txtSource, "Text"];
   Print["\tDone."];
   Return[{strHeaderBody, strBodyBody}];
];


End[];


EndPackage[];
