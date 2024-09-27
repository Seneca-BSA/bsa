# Course Site

This is the supplementary materials website for various courses.

## Update Instruction

This site is created using [MkDocs](https://www.mkdocs.org/).

The following instruction is tested in Ubunut 20. Windows instruction should be similar.

To update the content of this site:

1. Ensure python and pip is installed on your computer.
1. Install mkdocs.
    ```
    pip install mkdocs
    ```
1. Clone this project locally.
    ```
    git clone https://github.com/Seneca-BSA/bsa.git
    cd bsa
    ```
1. Make changes as necessary.
1. To preview your change locally:
    ```
    mkdocs serve
    ```
    Or for Windows:
    ```
    python -m mkdocs serve
    ```
1. Once all changes are done, create a pull request with meaningful comment for review.

### For Admin

1. Get the latest version of the content from remote.
    ```
    git pull
    ```
1. To deploy the latest local content to github page.
    ```
    mkdocs gh-deploy
    ```
    **NOTE:** This does not push to the working branch. It only push to the gh-pages branch which serve the content.