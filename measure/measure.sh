#!/bin/bash

set -e

main() {

  local -i n=10
  local outDir
  local force=false
  local arg

  while (($#)); do
    arg="$1"; shift

    case "$arg" in
      -n)
        n=$1; shift
        ;;
      --out-dir)
        outDir="$1"; shift
        ;;
      --force)
        force=true
        ;;
    esac
  done

  if (( n <= 0 )); then
    printf "Nothing to run (n = %d)" $n 1>&2
    exit 1
  elif [[ -z "$outDir" ]]; then
    printf "Output directory not specified (use --out-dir DIR)" 1>&2
    exit 1
  fi

  if $force; then
    rm -rf "$outDir"
  elif [[ -d "$outDir" ]]; then
    printf "Output directory already exitst (use --force)" 1>&2
    exit 2
  fi

  mkdir -p "$outDir"

  for i in `seq 0 $((n - 1))`; do
    for f in none cpu gpu; do
      local m=`printf %02d $i`
      local out=`realpath --relative-base "$PWD" "$outDir/$m-$f.log"`
      echo "Running bin/rigid_body --$f --no-render > $out"
      optirun bin/rigid_body --$f --no-render > "$outDir/$m-$f.log" || true
    done
  done

}

main "$@"
