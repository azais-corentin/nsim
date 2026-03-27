{
  pkgs,
  ...
}:
{
  # https://devenv.sh/packages/
  packages = with pkgs; [ ];

  languages = {
    rust = {
      enable = true;
      channel = "stable";
      targets = [ "wasm32-unknown-unknown" ];
    };
    python = {
      enable = true;
      uv = {
        enable = true;
      };
    };
  };
}
