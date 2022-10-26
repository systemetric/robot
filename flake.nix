{
  description = "A basic flake with a shell";
  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
  inputs.nixpkgs-minion.url = "github:Minion3665/nixpkgs";
  inputs.flake-utils.url = "github:numtide/flake-utils";

  outputs = { self, nixpkgs, flake-utils, nixpkgs-minion }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = nixpkgs.legacyPackages.${system};
        pkgs-minion = nixpkgs-minion.legacyPackages.${system};
      in
      {
        devShells.default = pkgs.mkShell {
          nativeBuildInputs = [
            (pkgs-minion.python3.withPackages (pyPkgs: with pyPkgs;
            [
              smbus2
              opencv3
              fake-rpi
              scipy
              wiringpi
            ]))
          ];
          buildInputs = [ ];
        };
      });
}
