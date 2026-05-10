# How to Contribute

If you have improvements, suggestions for new features or maybe just some
cosmetic changes, the following sections provide some hints on how to contribute
to the project. Thanks for your work on this project!

## Submitting Changes

For now, just open a pull request with this repository. Please make sure to
clearly describe the changes that you would like to make, including a
rationale for large modifications.

## Coding Style

When looking at the code, you may notice that it is pretty conservative with
respect to using (advanced) language features of C++. Probably, you will have
also noticed a rather extensive amount of comments. The primary reason for this
is a decision for a coding philosophy that more or less follows the
principles taught in the book *Practical C Programming* by Steve Oualline.

Briefly summarized, I strongly believe that the problems engineers need to
solve in (systems) programming are hard themselves. Hence, the solutions to
them should not be obstructed by code that is hard to understand. This is not
only a matter of other people (and probably yourself after some time)
understanding your programs without pain but also a matter of security and
safety of software systems. Ideally, code should read like a good novel, with
the comments guiding the reader through what is actually happening.
When writing code for this project, please make sure to follow the
general style of the code that is already present.

Some of the most important style guidelines are the following:
* First and foremost: comments. Every variable declaration should have a comment
  attached to it, saying what it is used for. The same holds for functions whose
  comments should include a documentation of return codes and input parameters.
  Similarly, the purpose of classes and source files should be explained in
  comments. Also, the code itself should be structured in paragraphs to
  increase readability. As usual, each paragraph should be preceded by a
  comment describing its purpose.
* Use the existing style of comment boxes that outline the layout of each
  code file.
* For the declaration of each function, provide a doxygen-style comment block
  that explains the function's purpose, its parameters, and its return value.
* Non-obvious code statements should be documented as well. Prominent examples
  for such statements are lock orders and accesses to device registers.
* Use sensible variable names. Although I personally like short names, single
  character names are not appropriate, except for iterator variables.
* Mind the horizontal limit of 80 columns per line.
* If line breaks are required to satisfy the 80 columns requirement, align
  multi-line parameter lists etc.
* Indentation is done with four spaces per level, no tabs.
* Put opening braces on the same line as the context they belong to.
* Reference and pointer operators should be aligned to the right.
* Put a space between keywords and following characters (e.g., opening
  parentheses)
* In a conditional block, the `else` keyword goes on a new line.
* For loops or conditional blocks that contain only a single line, the
  enclosing braces can be omitted.
* One-line conditional blocks or loops are forbidden.
* For switch constructs, indent the `case` labels. Indent code blocks
  following `case` labels one more level than the `case` label itself.

In particular, the following coding patterns should be avoided:
* The use of prefix / postfix operators in assignments (i.e., `var = i++;`).
* Extensive use of templates.
* Functional programming.
* Pointless layers of abstractions and inheritance.
* Preprocessor abuse.
* Overmodularization of code. It is ok to have functions longer than 10 lines.

Of course - apart from the guidelines on comments - there may be exceptions to
these rules. When there is a good reason to deviate from them, do so, but
leave a comment on why you did.
