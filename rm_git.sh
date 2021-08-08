for d in */ ; do
    cd $d
    if [ -d ".git" ]; then
        echo "$d git exist"
        rm -rf .git
    fi
    cd ..
done
