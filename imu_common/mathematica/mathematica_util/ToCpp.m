(* ::Package:: *)

BeginPackage["ToCpp`"];

Needs["ExtraUtil`"];

(*
@todo Inconsistency in a definition either having a symbol or a string... How to fix this?
@todo Ambiguous definition of definition as either an argument definition or a function definition...
@todo Make sure that only known symbols are present in an expression?
@todo Precision - especially for integers!
X For large integers, it's converting them to floating-point values... Bad!
- Actually, this is a little more complicated in C.
*)

ToCppLoadTemplate::usage =
"ToCppLoadTemplate[tplBase = <default>, includeDir = None]  load template base .h and .cpp to and return loaded template information
The <default> for tplBase is 'expressions.tpl' relative to this source file's path
includeDir - ONLY FOR TEMPLATE. If None, will be same directory as source file. Otherwise, will concat this dir and basename of source file";

ToCppFunc::usage =
"ToCppFunc[...]  generates C++ code with Eigen library for given expression.
argSet - {{argName, argType}, ...} in C++
Note that all fractions are 'numericized' and array indices (i.e. q[2]) are converted from 1..n to 0..n-1 (thus q[i] -> q[i-1])
preStatement - Statement inserted before. Good for Common Subexpression Elimination. You handle your own newlines and indentation.
Returns {function declaration, function definition}";

ToCppGenerateFiles::usage =
"ToCppGenerateFiles[...]  genenerate header and source file for C++ Eigen usage.
defSet = {arguments for ToCppFunc[...] - see help for this}
fileBase - basename for .h and .cpp files to be generated";

ToCppExpr::usage = "
ToCppExpr[...]  converts expr to C++ Eigen form or double assigned to variable name.";

CRoundIndexSubs::usage = 
"CRoundIndexSubs[symbols]  make substitution rules to round and subtract 1 from all 'function' indices of a given symbol. i.e. x[5] -> x[4], a[10, 7] -> a[9, 6]";

ToCppGenerateUnittest::usage =
"ToCppGenerateUnittest[argSet]  Generate a simple unittest";

ToCppSetContext::usage = "ToCppSetContext[context]  set context for expression conversion";


Begin["`Private`"];

ExtraUtil`ContextAlias[{"StringImplode", "ExportText", "PseudoSplice", "Indentify", "EnsureDirectoryExists"}];

defaultTplBase = FileNameJoin[{DirectoryName[$InputFileName], "expressions.tpl"}];

mainContext = "Global`";

ToCppSetContext[context_:Context[]] := Module[{},
  mainContext = context;
];

ToCppLoadTemplate[tplBaseIn_: None] := 
  Module[{tplBase, tplHeader, tplSource, tplTest},
   tplBase = tplBaseIn;
   If[tplBase === None,
     tplBase = defaultTplBase
   ];
   tplHeader = Import[tplBase <> ".hpp", "Text"];
   tplSource = Import[tplBase <> ".cpp", "Text"];
   tplTest = Import[tplBase <> ".gtest.cpp", "Text"];
   tplMex = Import[tplBase <> ".mex.cpp", "Text"];
   Return[{tplHeader, tplSource, tplTest, tplMex}];
   ];

(* @brief Get a check statement to ensure a consistent size for an input or output *)
getCheck[name_, ctype_] := Module[{},
  Return[""];
];

getCheck[name_, ctype_, dim_] := Module[{args, check},
  If[Length[dim] == 0,
    Return[getCheck[name, ctype]];
  ];
  args = Map[ToString, Join[{name}, dim]];
  check = "assert_size(" <> StringImplode[args, ", "] <> ");";
  Return[check];
];

getCheck[args_List] := getCheck@@args;

CRoundIndices[indices_List] := Table[If[NumberQ[index], Round[index] - 1, index], {index, indices}];
CRoundIndexSubs[symbols_List] := Map[#[indices__]:>#@@CRoundIndices[{indices}]&@#&, symbols];

ToCppExpr[name_String, expr_, symbols_List: {}, indent_: "    "] := 
  Module[{temp, strOut, type, dims, strSize, cforms, strForms, strExpr, 
    newLine, strExtra, indexSubs, context, exp},

    (* Apply substitutions *)
    (* - Replace E^x and Exp[x] with exp[x] before numericizing *)
    exp = ToExpression[mainContext <> "exp"];
    temp = expr /. {Exp[x_] :> exp[x], E^x_ :> exp[x]};
    (* - Numericize *)
    temp = N[temp];
    (* - Format Subindices and re-round them *)
    indexSubs = CRoundIndexSubs[symbols];
    temp = temp /. indexSubs;

    (* Format the expression to a simpler expression to be put into a list *)
    If[ListQ[temp],
        (* Flatten expression *)
        temp = Flatten[temp];
        (* Handle empty matrices *)
        If[Length[temp] > 0,
            cforms = Map[CForm, temp];
            strForms = Map[ToString, cforms];
            strExpr = StringImplode[strForms, ",\n"];    
            strOut = StringJoin[{name, " <<\n", Indentify[strExpr, indent], ";"}];
        ,
            strOut = "";
        ];
    ,
        (*Scalar*)
        strExpr = CForm[temp];
        strOut = StringImplode[{name, " = ", strExpr, ";"}];
    ];
    Return[strOut];
   ];

getExprDef[name_String, expr_] := Module[{},
    If[ListQ[expr],
        Return[getFullDef[{name, Dimensions[expr]}]];
    ,
        Return[getFullDef[{name}]];
    ];
];

(* @brief Take a simple argument definition of {name, sizes} and return {name, ctype, sizes} for simplicity *)
getFullDef[argMin_] := Module[{name, sizes, ndim},
    name = argMin[[1]];
    If[Length[argMin] == 3 || (Length[argMin] >=2 && StringQ[argMin[[2]]]),
        Return[argMin];
    ];
    If[Length[argMin] == 1,
        Return[{name, "double", {}}];
    ,
        sizes = argMin[[2]];
        If[!ListQ[sizes], sizes = {sizes}];
        ndim = Length[sizes];
        If[ndim == 1,
            Return[{name, "Eigen::VectorXd", sizes}];
        ,
            Assert[ndim == 2];
            Return[{name, "Eigen::MatrixXd", sizes}];
        ];
    ];
];

(* Easy version *)
ToCppFunc[funcName_String, argSetMin_List, expr_] :=
        Module[{output, argSet},
    (* Expand funcName using expr to output *)
    output = getExprDef[funcName, expr];
    (* Expand each definition based on the size supplied *)
    argSet = Map[getFullDef, argSetMin];
    (* Return the concise results *)
    Return[ToCppFunc[output, argSet, expr]];
];

(* Concise version *)
ToCppFunc[output_List, argSet_, expr_, preStatement_: "", doChecks_: True, indent_: "    "] := 
        Module[{strVar, strExpr, ctype, strDecl, strDef, args, strArgs, indexSubs, decl},

    strVar = "output_value";
    funcName = output[[1]];
    ctype = output[[2]];
    sizes = output[[3]];
    (* Get resize statement *)
    If[Length[sizes] > 0,
        (* Use resize() so that a malloc() disabling can be triggered if needed *)
        check = strVar <> ".resize(" <> StringImplode[sizes, ", "] <> ");";
    ,
        check = "";
    ];
    symbols = argSet[[;;, 1]];
    strExpr = ToCppExpr[strVar, expr, symbols, indent];
    checks = {check};
    (* Go through each input and generate checks and the C++ function argument *)
    args = {};
    Table[
        check = getCheck[arg];
        If[StringLength[check] > 0,
            AppendTo[checks, check];
        ];
        AppendTo[args, StringJoin["const ", arg[[2]], " &", ToString[arg[[1]]]]];
    ,
        {arg, argSet}
    ];
    (* Add output argument *)
    AppendTo[args, StringJoin[ctype, " &", strVar]];
    strArgs = StringImplode[args, ", "];
    If[!doChecks,
        checks = {};
    ];
    strDecl = StringJoin["void " , funcName, "(", strArgs, ")"];
    lines = {preStatement, checks, strExpr};
    strBody = StringImplode[Flatten[lines], "\n"];
    strDef = StringJoin[strDecl, "\n{\n", Indentify[strBody, indent], "\n}"];
    strTest = ToCppUnittest[output, argSet, expr];
    subsMex = ToCppMex[output, argSet];
    Return[{strDecl, strDef, strTest, subsMex}];
  ];

getDecl[arg_, subfix_: "", valueIn_: None] := 
    Module[{name, ctype, sizes, var, decl, value, check, cpp, subs},
    value = valueIn;
    (* Poorly scoped super function *)
    {name, ctype, sizes} = arg;
    var = ToString[name] <> subfix;
    If[Length[sizes] == 0,
        decl = ctype <> " " <> var <> ";";
        If[value === None, value = rand[]];
        subs = name -> value;
    ,
        decl = ctype <> " " <> var <> "(" <> StringImplode[Map[ToString, sizes], ", "] <> ");";
        If[value === None, value = Array[rand, sizes]];
        subs = name[indices__] :> Part[value, indices];
    ];
    cpp = ToCppExpr[var, value, {}];
    Return[{decl, cpp, var, value, subs}];
];

(* Avoid all zeros *)
rand[] := Module[{update, temp},
   update[] := (temp = Random[Real, {-20, 20}]);
   update[];
   While[temp == 0, update[]];
   Return[temp];
];
rand[meh__] := rand[];

ToCppMex[outputIn_, argSet_] := Module[
        {output, name, ctype, sizes, inputDecl, outputDecl, symbols, args, outputInvoke},
    {name, ctype, sizes} = outputIn;
    output = outputIn;
    output[[1]] = "output_value";
    inputDecl = Map[getDecl, argSet];
    outputDecl = getDecl[output];
    symbols = argSet[[;; , 1]];
    args = Append[Map[ToString, symbols], outputDecl[[3]]];
    argsPass = StringImplode[args, ", "];

    indent = "        ";
    nargin = Length[argSet];
    inputDeclLines = {};
    Table[
        AppendTo[inputDeclLines, inputDecl[[i, 1]]];
        AppendTo[inputDeclLines, "from_mex(argin[" <> ToString[i - 1] <> "], " <> ToString[symbols[[i]]] <> ");"];
    ,
        {i, nargin}
    ];
    inputDeclStr = StringImplode[inputDeclLines, "\n"];

    extsubs = {
        "EXPR_IN_DECL" -> Indentify[inputDeclStr, indent],
        "EXPR_OUT_DECL" -> Indentify[outputDecl[[1]], indent],
        "EXPR_ARGS_PASS" -> argsPass,
        "EXPR_NARGIN" -> ToString[nargin],
        "EXPR_NAME" -> name
    };
    Return[extsubs];
];

(* Generate a gtest body for a given function definition *)
ToCppUnittest[output_, argSet_, expr_] := Module[
        {getCheck, subs,
        name, ctype, sizes, inputDecl, outputDeclExpected, outputDeclActual, symbols, outputArgs, outputInvoke, outputCheck, lines},
    {name, ctype, sizes} = output;

    getCheck[arg_] := Module[{var, ctype, sizes, check},
        {var, ctype, sizes} = arg;
        check = 
          ctype <> " " <> var <> "_diff = diff_relative_nonzero(" <> var <> "_actual, " <> 
           var <> "_expected);\nEXPECT_LT(norm_nanless(" <> var <> 
           "_diff), tolerance);";
        Return[check];
    ];

    inputDecl = Map[getDecl, argSet];
    subs = inputDecl[[;;, 5]];
    expectedValue = (expr /. subs) // N;
    outputDeclExpected = getDecl[output, "_expected", expectedValue];
    outputDeclActual = getDecl[output, "_actual"];
    symbols = argSet[[;; , 1]];
    outputArgs = Append[Map[ToString, symbols], outputDeclActual[[3]]];
    outputInvoke = StringJoin[{name, "(", StringImplode[outputArgs, ", "], ");"}];
    outputCheck = getCheck[output];

    lines = {};
    Table[
        JoinTo[lines, decl[[{1, 2}]]]
    ,
        {decl, inputDecl}
    ];
    JoinTo[lines, outputDeclExpected[[{1, 2}]]];
    AppendTo[lines, outputDeclActual[[1]]];
    JoinTo[lines, {outputInvoke, outputCheck}];

    Return[StringImplode[lines, "\n"]];
];


ToCppGenerateFiles[template_List, headerDir_String, sourceDir_String, fileName_String, namespace_String, includeDir_:None, defSet_List, testDirIn_:None, doWrite_:True, verbose_:False] := 
  Module[{pair, pairs, strHeaders, strSources, strHeaderBody, 
    strSourceBody, subs, fileHeader, fileSource, fileBaseTrue, 
    fileHeaderForTemplate, tplHeader, tplSource, tplMex, testDir, mexDir}, 

    If[verbose,
        print[x_] := Print[x];
    ,
        print[x_] := None;
    ];

    testDir = testDirIn;
    If[testDir === None,
      testDir = sourceDir;
    ];

    mexDir = FileNameJoin[{sourceDir, "..", "mex"}];

    (* Extract template stuff *)
    {tplHeader, tplSource, tplTest, tplMex} = template;
   print["C++ Conversion:"];
   pairs = Table[
        print["\t\t" <> def[[1]]];
        pair = ToCppFunc @@ def;
        pair
   ,
        {def, defSet}
   ];
   {strHeaders, strSources, strTests, subMexs} = Transpose[pairs];
   strHeaderBody = StringImplode[strHeaders, ";\n\n"] <> ";";
   strSourceBody = StringImplode[strSources, "\n\n"];
   strTestsIndented = Table["{\n" <> Indentify[strTest] <> "\n}", {strTest, strTests}];
   strTestBody = StringImplode[strTestsIndented, "\n"];
   fileHeader = FileNameJoin[{headerDir, fileName <> ".hpp"}];
   fileSource = FileNameJoin[{sourceDir, fileName <> ".cpp"}];
   fileTest = FileNameJoin[{testDir, "test_" <> fileName <> ".cpp"}];
   If[includeDir === None,
     fileHeaderForTemplate = fileName <> ".hpp";
   , 
     fileHeaderForTemplate = FileNameJoin[{includeDir, fileName}] <> ".hpp";
   ];
   subs = {
        {"EXPR_HEADER", fileHeaderForTemplate},
        {"EXPR_INCLUDE_GUARD", "_H_" <> ToUpperCase[namespace <> "_" <> StringReplace[fileName, "." -> "_"]]},
        {"EXPR_NAMESPACE", namespace},
        {"EXPR_DECLARATIONS", strHeaderBody},
        {"EXPR_DEFINITIONS", strSourceBody},
        {"EXPR_TESTS", Indentify[strTestBody]}
    };
   txtHeader = PseudoSplice[tplHeader, subs];
   txtSource = PseudoSplice[tplSource, subs];
   txtTest = PseudoSplice[tplTest, subs];
   print["\t", fileHeader, "\n\t", fileSource];
   If[doWrite,
       Map[EnsureDirectoryExists, {headerDir, sourceDir, testDir, mexDir}];
       Export[fileHeader, txtHeader, "Text"];
       Export[fileSource, txtSource, "Text"];
       Export[fileTest, txtTest, "Text"];
       Table[
            name = defSet[[i, 1]];
            subMex = subMexs[[i]];
            fileMex = FileNameJoin[{mexDir, name <> "_mex.cpp"}];
            txtMex = PseudoSplice[tplMex, Join[subs, subMex]];
            Export[fileMex, txtMex, "Text"];
       ,
           {i, Length[pairs]}
       ];
   ];
   print["\tDone."];

   Return[{{fileHeader, txtHeader}, {fileSource, txtSource}, {fileTest, txtTest}}];
];

End[];


EndPackage[];
