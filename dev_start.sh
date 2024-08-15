if test "$(docker images -q mola:mola 2> /dev/null)" = ""
then
    echo "build mola image"
    docker compose build
else
    docker compose down
    docker compose up -d mola
fi

