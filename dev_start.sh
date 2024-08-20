if test "$(docker images -q modular_slam:modular_slam 2> /dev/null)" = ""
then
    echo "build modular_slam image"
    docker compose build
else
    docker compose down
    docker compose up -d modular_slam
fi

